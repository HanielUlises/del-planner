#include "search.hpp"

#include "bisimulation.hpp"
#include "parallel.hpp"
#include "product_update.hpp"
#include "symmetry.hpp"
#include "world_cap_policy.hpp"

#include <algorithm>
#include <unistd.h>
#include <chrono>
#include <climits>
#include <deque>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Search
//
// Three representation invariants are shared by every algorithm in this file.
//
// A state is stored exactly once. Nodes live in a deque arena and the open list
// holds (h, g, index) triples of twelve bytes, so heap sift operations permute
// integers rather than Kripke models. Holding nodes by value in the priority
// queue would instead deep-copy a model on every sift and on every expansion.
//
// Plans are represented as parent links. Each node carries its parent's index
// and the action that reached it, and the action sequence is reconstructed once
// on success. Carrying the prefix in each node would make plan storage alone
// O(nodes · depth).
//
// Closed lists store 128-bit fingerprints rather than states. Contraction
// assigns a canonical world numbering, so fingerprint equality coincides with
// bisimilarity: two models representing the same epistemic situation under
// different world labellings fingerprint identically and are not re-expanded. A
// hash sensitive to world numbering would not have that property, and would
// re-expand states already closed.



namespace {
thread_local const std::atomic<bool>* t_cancel = nullptr;
} // namespace

void set_cancel_flag(const std::atomic<bool>* flag) noexcept { t_cancel = flag; }

bool expired(Deadline d) noexcept {
    return (t_cancel && t_cancel->load(std::memory_order_relaxed)) ||
           std::chrono::steady_clock::now() >= d;
}

namespace {

// Stabiliser of a canonical state under the task's agent swaps.
StateSymmetry symmetry_of(const PlanningTask& task, const EpistemicState& s) {
    return task.symmetry ? stabiliser(*task.symmetry, s) : StateSymmetry{};
}

// False for an action that is not its orbit's representative at this state.
bool keep(const PlanningTask& task, const StateSymmetry& stab, ActionIdx ai,
          PlannerStats& stats) {
    if (stab.trivial || stab.keep(*task.symmetry, ai)) return true;
    ++stats.pruned_symmetric;
    return false;
}

} // namespace

namespace {

constexpr std::uint32_t kNoNode = std::numeric_limits<std::uint32_t>::max();

using FingerprintSet = std::unordered_set<Fingerprint, FingerprintHash>;

// One successor, built without touching shared search state except for
// read-only lookups in `closed`.
struct Successor {
    bool           applicable{false};
    PruneReason    pruned{PruneReason::None};
    bool           goal{false};
    float          h{0.f};
    Fingerprint    fp;
    EpistemicState state;
    CompactState   compact;        // what the queue stores
};

void build_successor(const EpistemicState& parent, const Action& action,
                     const PlanningTask& task, const Heuristic& h,
                     const WorldCapPolicy& cap, const FingerprintSet& closed,
                     Successor& out) {
    if (!action.applicable(parent)) return;
    out.applicable = true;

    auto maybe = product_update(parent, action, task.frame_guard(), cap);
    if (!maybe) { out.pruned = maybe.error(); return; }

    out.state = bisim_contract(std::move(*maybe));
    out.fp    = out.state.fingerprint();
    out.goal  = out.state.satisfies(*task.goal);
    if (!out.goal && !closed.contains(out.fp)) {
        out.h       = h(out.state, task);
        out.compact = CompactState::from(out.state);
    }
    out.state = EpistemicState{};
}

// Successors built at once without exceeding half the free memory. A product
// update keeps |W|·|E| worlds before contraction; bound its set table by one
// successor list of every world per agent, plus contraction's working keys.
std::size_t parallel_batch(const EpistemicState& s, const PlanningTask& task) {
    static std::size_t max_events = [&] {
        std::size_t m = 1;
        for (const Action& a : task.actions) m = std::max(m, a.events.size());
        return m;
    }();
    const double worlds = double(s.num_worlds) * double(max_events);
    const double per_world = double(s.members.size()) / double(std::max<std::uint32_t>(1, s.num_worlds)) + 1.0;
    const double bytes  = 4.0 * worlds * double(std::max<std::uint32_t>(1, s.num_agents)) * (per_world + 4.0) * 3.0;
    const double avail  = 0.5 * double(sysconf(_SC_AVPHYS_PAGES)) * double(sysconf(_SC_PAGESIZE));
    const double batch  = bytes > 0 ? avail / bytes : double(par::threads());
    return std::size_t(std::clamp(batch, 1.0, double(par::threads())));
}

// Enough work per expansion to cover waking the pool.
bool worth_parallel(const EpistemicState& s, std::size_t actions) {
    return par::threads() > 1 && actions >= 4 &&
           std::size_t(s.num_worlds) * actions >= 4096;
}

// Computes every extension successor generation reads, so that concurrent
// sat() calls on `s` only look up.
void warm_extensions(const EpistemicState& s, const PlanningTask& task) {
    for (const Action& a : task.actions) {
        for (const Event& e : a.events) {
            (void)s.sat(*e.precondition);
            for (const auto& [atom, cond] : e.post_true)  (void)s.sat(*cond);
            for (const auto& [atom, cond] : e.post_false) (void)s.sat(*cond);
        }
        for (const auto& cases : a.obs_cases)
            for (const ObsCase& c : cases) (void)s.sat(*c.condition);
    }
    (void)s.fingerprint();
}

// Reconstruct an action-name sequence by walking parent links to the root.
template <class NodeArena>
std::vector<std::string> reconstruct(const NodeArena& nodes, std::uint32_t leaf,
                                     const PlanningTask& task) {
    std::vector<std::string> plan;
    for (std::uint32_t i = leaf; i != kNoNode && nodes[i].parent != kNoNode;
         i = nodes[i].parent)
        plan.push_back(task.actions[nodes[i].action].name);
    std::reverse(plan.begin(), plan.end());
    return plan;
}

} // namespace

namespace gbfs {

namespace {

struct Node {
    CompactState   state;          // released once expanded
    std::uint32_t  parent{kNoNode};
    ActionIdx      action{0};
    std::uint32_t  g{0};
};

struct QEntry {
    float         h;
    std::uint32_t g;
    std::uint32_t idx;
};

// std::push_heap builds a max-heap under the comparator, so this reports
// "a is worse than b": higher h first, and among equal h the *shallower* node,
// which leaves the deeper one on top. Diving on ties is the standard greedy
// tie-break and matters here because epistemic plateaus are wide.
struct Worse {
    bool operator()(const QEntry& a, const QEntry& b) const noexcept {
        if (a.h != b.h) return a.h > b.h;
        return a.g < b.g;
    }
};

} // namespace

namespace {

std::optional<SearchResult> run(const PlanningTask& task, const Heuristic& h,
                                std::size_t max_nodes, Deadline deadline,
                                bool helpful, bool& pruned) {
    SearchResult result;
    result.stats.start_timer();

    const WorldCapPolicy cap = make_world_cap_policy(task.partial_obs);

    EpistemicState init = bisim_contract(task.init);
    if (init.satisfies(*task.goal)) {
        result.stats.stop_timer();
        return result;
    }

    result.stats.heuristic_calls++;
    const float init_h = h(init, task);
    result.stats.initial_h = init_h;
    result.stats.best_h    = init_h;
    result.stats.final_h   = init_h;

    std::deque<Node>    nodes;
    std::vector<QEntry> open;
    FingerprintSet      closed;
    std::size_t         live_bytes = 0;

    closed.insert(init.fingerprint());
    live_bytes += init.footprint();
    nodes.push_back(Node{CompactState::from(init), kNoNode, 0, 0});
    open.push_back(QEntry{init_h, 0, 0});

    while (!open.empty()) {
        std::pop_heap(open.begin(), open.end(), Worse{});
        const QEntry cur = open.back();
        open.pop_back();

        result.stats.nodes_expanded++;

        if (max_nodes > 0 && result.stats.nodes_expanded > max_nodes) {
            std::cerr << "[gbfs] Node limit reached (" << max_nodes << ").\n";
            result.stats.stop_timer();
            return std::nullopt;
        }
        if (expired(deadline)) {
            std::cerr << "[gbfs] Deadline exceeded at "
                      << result.stats.nodes_expanded << " nodes.\n";
            result.stats.stop_timer();
            return std::nullopt;
        }

        // Copy the state out before generating successors: pushing into `nodes`
        // can reallocate the deque's index block, and holding a reference to an
        // element across that would be fragile even though deque never moves
        // its elements.
        const std::uint32_t cur_idx = cur.idx;
        bool generated_successor = false;

        const EpistemicState parent = nodes[cur_idx].state.expand();
        const StateSymmetry   stab   = symmetry_of(task, parent);

        std::vector<ActionIdx> cands;
        for (ActionIdx ai = 0; ai < task.actions.size(); ++ai)
            if (keep(task, stab, ai, result.stats)) cands.push_back(ai);

        // Helpful actions first: expand only relaxed-plan actions when there
        // are any; a failed search that pruned this way is retried in full.
        if (helpful) {
            thread_local std::vector<ActionIdx> pref;
            if (h.preferred(parent, task, pref)) {
                std::vector<ActionIdx> kept;
                for (ActionIdx ai : cands)
                    if (std::binary_search(pref.begin(), pref.end(), ai)) kept.push_back(ai);
                if (!kept.empty() && kept.size() < cands.size()) {
                    cands.swap(kept);
                    pruned = true;
                }
            }
        }

        // Successors are built independently, in parallel when the models are
        // large enough to pay for it, and merged in action order so results do
        // not depend on the thread count.
        std::vector<Successor> succ(cands.size());
        const auto build = [&](std::size_t k) {
            build_successor(parent, task.actions[cands[k]], task, h, cap, closed, succ[k]);
        };
        const bool parallel = worth_parallel(parent, cands.size());
        if (parallel) {
            warm_extensions(parent, task);
            const std::size_t batch = parallel_batch(parent, task);
            for (std::size_t at = 0; at < cands.size(); at += batch) {
                if (expired(deadline)) break;
                par::for_each_index(std::min(batch, cands.size() - at),
                                    [&](std::size_t i) { build(at + i); });
            }
        }

        for (std::size_t k = 0; k < cands.size(); ++k) {
            if (expired(deadline)) {
                std::cerr << "[gbfs] Deadline exceeded at "
                          << result.stats.nodes_expanded << " nodes.\n";
                result.stats.stop_timer();
                return std::nullopt;
            }
            if (!parallel) build(k);
            Successor& sc = succ[k];
            if (!sc.applicable) continue;
            if (sc.pruned != PruneReason::None) { result.stats.record_prune(sc.pruned); continue; }

            const ActionIdx ai = cands[k];
            generated_successor = true;
            result.stats.nodes_generated++;

            const std::uint32_t g = nodes[cur_idx].g + 1;

            if (sc.goal) {
                nodes.push_back(Node{CompactState{}, cur_idx, ai, g});
                result.plan = reconstruct(nodes, static_cast<std::uint32_t>(nodes.size() - 1), task);
                result.stats.final_h    = 0.f;
                result.stats.closed_size = closed.size();

                std::cerr << "[gbfs] Solution found. Length=" << result.plan.size()
                          << "  Expanded=" << result.stats.nodes_expanded
                          << "  Generated=" << result.stats.nodes_generated
                          << "  Closed=" << closed.size()
                          << "  PeakStateBytes=" << result.stats.peak_state_bytes
                          << "\n";

                result.stats.stop_timer();
                return result;
            }

            // A sibling built in parallel may duplicate this one; h is skipped
            // only for states already closed when the successor was built.
            if (!closed.insert(sc.fp).second) {
                result.stats.duplicates_pruned++;
                continue;
            }

            result.stats.heuristic_calls++;
            const float hv = sc.h;
            result.stats.final_h = hv;
            if (hv < result.stats.best_h) {
                result.stats.best_h = hv;
                result.stats.heuristic_improvements++;
            } else {
                result.stats.heuristic_stalls++;
            }

            live_bytes += sc.compact.footprint();
            result.stats.peak_state_bytes =
                std::max(result.stats.peak_state_bytes, live_bytes);

            nodes.push_back(Node{std::move(sc.compact), cur_idx, ai, g});
            open.push_back(QEntry{hv, g, static_cast<std::uint32_t>(nodes.size() - 1)});
            std::push_heap(open.begin(), open.end(), Worse{});

            result.stats.max_frontier_size =
                std::max(result.stats.max_frontier_size, open.size());
        }

        // Expanded: only the parent link and action are needed from here on.
        live_bytes -= nodes[cur_idx].state.footprint();
        nodes[cur_idx].state = CompactState{};

        if (!generated_successor) result.stats.dead_ends++;
    }

    result.stats.closed_size = closed.size();
    std::cerr << (pruned ? "[gbfs] Helpful-action search exhausted.\n"
                         : "[gbfs] Search exhausted — no solution.\n");
    result.stats.stop_timer();
    return std::nullopt;
}

} // namespace

std::optional<SearchResult> search(const PlanningTask& task, const Heuristic& h,
                                   std::size_t max_nodes, Deadline deadline) {
    bool pruned = false;
    auto r = run(task, h, max_nodes, deadline, task.helpful_actions, pruned);
    if (r || !pruned || expired(deadline)) return r;
    std::cerr << "[gbfs] Retrying without helpful-action pruning.\n";
    return run(task, h, max_nodes, deadline, false, pruned);
}

} // namespace gbfs

namespace aostar {

namespace {

// One expanded action: the contracted state of every sensing outcome, plus an
// aggregate heuristic estimate.
//
// The split is computed once and carried into the recursion. Ranking candidates
// by an unsplit product_update and re-splitting the committed action would
// evaluate the dominant cost of the expansion twice.
//
// Ranking uses the worst branch rather than the heuristic of the merged product.
// An AND node is solved only when every branch is solved, so the binding
// constraint is the hardest outcome; the merged product's designated set is the
// union over designated events and therefore describes no branch in
// particular.
struct Expansion {
    ActionIdx                                       action{0};
    float                                           h{0.f};
    std::vector<std::pair<EventIdx, EpistemicState>> branches;
};

struct Solved {
    std::shared_ptr<PlanNode> tree;     // null = state already satisfies the goal
    std::uint32_t             height{0};
};

struct DfsResult {
    bool                      solved{false};
    std::shared_ptr<PlanNode> tree;
    std::uint32_t             height{0};

    // True when this failure depended on the ancestor cut or on the deadline
    // rather than on the depth bound alone. Such failures are path- or
    // time-dependent and must not enter the memo: the same state reached by a
    // different path, or at a later moment, may well be solvable.
    bool tainted{false};
};

// A refuted state, and whether that refutation was forced by the depth bound.
struct Refuted {
    std::uint32_t depth{0};       // greatest depth at which failure was proven
    bool          truncated{false};
};

struct Context {
    const PlanningTask& task;
    const Heuristic&    h;
    WorldCapPolicy      cap;
    PlannerStats&       stats;
    Deadline            deadline;

    // Set whenever a branch was abandoned because the depth budget ran out.
    // If a whole iteration completes without this being set, the depth bound
    // never bound anything: the search visited the entire reachable AND-OR
    // space and a larger budget cannot reach further. That is a proof of
    // unsolvability, not a reason to try depth+1.
    //
    // Without it the planner spins forever on unsolvable tasks — cn-2 under a
    // sensing encoding has two worlds and reaches depth 1.5 million. The
    // previous test compared expansion counts between iterations, which the
    // persistent memo defeats: memo hits keep each iteration cheap but still
    // expand more than the root.
    bool truncated{false};

    // Both memos persist across the iterative-deepening iterations. The previous
    // implementation rebuilt its memo table at every depth, so each iteration
    // re-expanded from scratch everything the last one had already refuted.
    std::unordered_map<Fingerprint, Solved, FingerprintHash>   solved;
    std::unordered_map<Fingerprint, Refuted, FingerprintHash>  failed_upto;

    FingerprintSet ancestors;
};

std::vector<Expansion> expand(const EpistemicState& s, Context& ctx) {
    std::vector<Expansion> out;
    out.reserve(ctx.task.actions.size());

    const StateSymmetry stab = symmetry_of(ctx.task, s);

    for (ActionIdx ai = 0; ai < ctx.task.actions.size(); ++ai) {
        const Action& a = ctx.task.actions[ai];
        if (!keep(ctx.task, stab, ai, ctx.stats)) continue;
        if (!a.applicable(s)) continue;

        auto branches = product_update_split(s, a, ctx.task.frame_guard(), ctx.cap);
        if (branches.empty()) continue;

        Expansion e;
        e.action = ai;
        e.branches.reserve(branches.size());

        float worst = 0.f;
        for (auto& [eid, bstate] : branches) {
            EpistemicState contracted = bisim_contract(std::move(bstate));
            ctx.stats.heuristic_calls++;
            worst = std::max(worst, ctx.h(contracted, ctx.task));
            e.branches.emplace_back(eid, std::move(contracted));
        }

        e.h = worst;
        ctx.stats.nodes_generated += e.branches.size();
        ctx.stats.best_h = std::min(ctx.stats.best_h, worst);
        out.push_back(std::move(e));
    }

    std::sort(out.begin(), out.end(), [](const Expansion& x, const Expansion& y) {
        if (x.h != y.h) return x.h < y.h;
        return x.action < y.action;      // deterministic tie-break
    });
    return out;
}

DfsResult dfs(const EpistemicState& s, std::size_t depth, Context& ctx) {
    ctx.stats.nodes_expanded++;

    if (expired(ctx.deadline)) {
        // Also counts as truncation: the iteration did not finish, so its
        // failure is no evidence that the space was searched.
        ctx.truncated = true;
        return DfsResult{false, nullptr, 0, true};
    }

    const Fingerprint fp = s.fingerprint();

    if (ctx.ancestors.count(fp))
        return DfsResult{false, nullptr, 0, true};

    if (s.satisfies(*ctx.task.goal))
        return DfsResult{true, nullptr, 0, false};

    // A cached solution is reusable only if it fits the remaining budget;
    // otherwise the depth bound this iteration is enforcing would be violated.
    if (auto it = ctx.solved.find(fp); it != ctx.solved.end() &&
                                       it->second.height <= depth)
        return DfsResult{true, it->second.tree, it->second.height, false};

    if (depth == 0) {
        ctx.truncated = true;               // the bound, not the domain, stopped us
        return DfsResult{false, nullptr, 0, false};
    }

    // Failure at depth d implies failure at every d' ≤ d. If that failure was
    // itself forced by the bound, replaying it must re-raise the flag —
    // conservatively, since a truncated failure at depth d is also truncated at
    // any smaller depth. Erring this way can only delay the exhaustion proof,
    // never fabricate one.
    if (auto it = ctx.failed_upto.find(fp); it != ctx.failed_upto.end() &&
                                            depth <= it->second.depth) {
        if (it->second.truncated) ctx.truncated = true;
        return DfsResult{false, nullptr, 0, false};
    }

    std::vector<Expansion> candidates = expand(s, ctx);
    if (candidates.empty()) ctx.stats.dead_ends++;

    ctx.ancestors.insert(fp);
    const bool truncated_before = ctx.truncated;
    bool tainted = false;

    for (const Expansion& e : candidates) {
        auto node = std::make_shared<PlanNode>();
        node->action = ctx.task.actions[e.action].name;
        node->branches.reserve(e.branches.size());

        bool          all_ok = true;
        std::uint32_t height = 0;

        for (const auto& [eid, branch] : e.branches) {
            const DfsResult r = dfs(branch, depth - 1, ctx);
            tainted |= r.tainted;
            if (!r.solved) { all_ok = false; break; }
            node->branches.emplace_back(eid, r.tree);
            height = std::max(height, r.height);
        }

        if (all_ok) {
            ctx.ancestors.erase(fp);
            const std::uint32_t h = height + 1;
            ctx.solved[fp] = Solved{node, h};
            return DfsResult{true, node, h, false};
        }
    }

    ctx.ancestors.erase(fp);

    if (!tainted) {
        Refuted& rec = ctx.failed_upto[fp];
        const auto d = static_cast<std::uint32_t>(depth);
        if (d >= rec.depth) {
            rec.depth     = d;
            rec.truncated = ctx.truncated && !truncated_before;
        }
    }
    return DfsResult{false, nullptr, 0, tainted};
}

} // namespace

std::optional<ConditionalSearchResult>
search(const PlanningTask& task, const Heuristic& h,
       std::size_t max_depth, Deadline deadline, bool* exhausted) {

    ConditionalSearchResult out;
    out.stats.start_timer();

    EpistemicState init = bisim_contract(task.init);

    if (init.satisfies(*task.goal)) {
        out.stats.stop_timer();
        return out;
    }

    out.stats.heuristic_calls++;
    const float init_h = h(init, task);
    out.stats.initial_h = init_h;
    out.stats.best_h    = init_h;
    out.stats.final_h   = init_h;

    Context ctx{task, h, make_world_cap_policy(task.partial_obs), out.stats, deadline,
                false, {}, {}, {}};

    const std::size_t depth_limit = (max_depth == 0) ? SIZE_MAX : max_depth;

    for (std::size_t depth = 0; depth <= depth_limit; ++depth) {
        if (expired(deadline)) {
            std::cerr << "[aostar] Timeout at depth " << depth << ".\n";
            out.stats.stop_timer();
            return std::nullopt;
        }

        std::cerr << "[aostar] Trying depth " << depth << "\n";
        ctx.ancestors.clear();
        ctx.truncated = false;

        const DfsResult r = dfs(init, depth, ctx);

        if (r.solved) {
            out.plan_tree = r.tree;
            std::cerr << "[aostar] Solution found at depth " << depth
                      << "  Expanded=" << out.stats.nodes_expanded
                      << "  Generated=" << out.stats.nodes_generated
                      << "  Memo=" << ctx.solved.size() << "/" << ctx.failed_upto.size()
                      << "\n";
            out.stats.stop_timer();
            return out;
        }

        // Nothing was cut off by the budget, so the whole reachable AND-OR
        // space was searched and refuted. No deeper iteration can differ.
        if (!ctx.truncated) {
            std::cerr << "[aostar] Search space exhausted at depth " << depth
                      << " — no solution exists.\n";
            if (exhausted) *exhausted = true;
            out.stats.stop_timer();
            return std::nullopt;
        }
    }

    std::cerr << "[aostar] No solution within depth " << depth_limit << ".\n";
    out.stats.stop_timer();
    return std::nullopt;
}

} // namespace aostar

namespace ehc {

namespace {

struct Node {
    EpistemicState state;
    std::uint32_t  parent{kNoNode};
    ActionIdx      action{0};
    std::uint32_t  g{0};
};

} // namespace

std::optional<SearchResult> search(const PlanningTask& task, const Heuristic& h,
                                   std::size_t max_nodes, Deadline deadline) {
    SearchResult result;
    result.stats.start_timer();

    const WorldCapPolicy cap = make_world_cap_policy(task.partial_obs);

    EpistemicState init = bisim_contract(task.init);
    if (init.satisfies(*task.goal)) {
        result.stats.stop_timer();
        return result;
    }

    result.stats.heuristic_calls++;
    float cur_h = h(init, task);
    result.stats.initial_h = cur_h;
    result.stats.best_h    = cur_h;
    result.stats.final_h   = cur_h;

    // One arena and one visited set for both phases. Sharing the visited set is
    // what keeps the greedy descent from re-entering a region the plateau escape
    // already crossed, and vice versa.
    std::deque<Node> nodes;
    FingerprintSet   visited;

    visited.insert(init.fingerprint());
    nodes.push_back(Node{std::move(init), kNoNode, 0, 0});
    std::uint32_t cur_idx = 0;

    const auto budget_exhausted = [&] {
        if (max_nodes > 0 && result.stats.nodes_expanded > max_nodes) {
            std::cerr << "[ehc] Node limit reached (" << max_nodes << ").\n";
            return true;
        }
        if (expired(deadline)) {
            std::cerr << "[ehc] Deadline exceeded.\n";
            return true;
        }
        return false;
    };

    const auto finish = [&](std::uint32_t leaf) {
        result.plan = reconstruct(nodes, leaf, task);
        result.stats.final_h    = 0.f;
        result.stats.closed_size = visited.size();
        result.stats.stop_timer();
    };

    for (;;) {
        if (budget_exhausted()) { result.stats.stop_timer(); return std::nullopt; }

        struct Succ {
            EpistemicState state;
            ActionIdx      action;
            float          hval;
        };
        std::vector<Succ> succs;

        const StateSymmetry stab = symmetry_of(task, nodes[cur_idx].state);

        for (ActionIdx ai = 0; ai < task.actions.size(); ++ai) {
            const Action& action = task.actions[ai];
            if (!keep(task, stab, ai, result.stats)) continue;
            if (!action.applicable(nodes[cur_idx].state)) continue;

            auto maybe = product_update(nodes[cur_idx].state, action, task.frame_guard(), cap);
            if (!maybe) { result.stats.record_prune(maybe.error()); continue; }

            EpistemicState next = bisim_contract(std::move(*maybe));
            result.stats.nodes_generated++;

            if (next.satisfies(*task.goal)) {
                nodes.push_back(Node{std::move(next), cur_idx, ai, nodes[cur_idx].g + 1});
                result.stats.nodes_expanded++;
                finish(static_cast<std::uint32_t>(nodes.size() - 1));
                std::cerr << "[ehc] Solution found. Length=" << result.plan.size()
                          << "  Expanded=" << result.stats.nodes_expanded
                          << "  Generated=" << result.stats.nodes_generated << "\n";
                return result;
            }

            result.stats.heuristic_calls++;
            const float hv = h(next, task);
            result.stats.final_h = hv;
            if (hv < result.stats.best_h) {
                result.stats.best_h = hv;
                result.stats.heuristic_improvements++;
            } else {
                result.stats.heuristic_stalls++;
            }

            succs.push_back(Succ{std::move(next), ai, hv});
        }

        std::sort(succs.begin(), succs.end(), [](const Succ& a, const Succ& b) {
            if (a.hval != b.hval) return a.hval < b.hval;
            return a.action < b.action;
        });

        bool improved = false;
        for (Succ& s : succs) {
            if (s.hval >= cur_h) break;
            if (!visited.insert(s.state.fingerprint()).second) {
                result.stats.duplicates_pruned++;
                continue;
            }

            nodes.push_back(Node{std::move(s.state), cur_idx, s.action,
                                 nodes[cur_idx].g + 1});
            cur_idx  = static_cast<std::uint32_t>(nodes.size() - 1);
            cur_h    = s.hval;
            improved = true;
            result.stats.nodes_expanded++;
            break;
        }

        if (improved) continue;

        // Plateau escape.
        std::cerr << "[ehc] Plateau at h=" << cur_h << " — BFS escape...\n";

        std::queue<std::uint32_t> frontier;
        frontier.push(cur_idx);

        bool escaped = false;
        while (!frontier.empty() && !escaped) {
            const std::uint32_t node_idx = frontier.front();
            frontier.pop();

            result.stats.nodes_expanded++;
            if (budget_exhausted()) { result.stats.stop_timer(); return std::nullopt; }

            const StateSymmetry stab = symmetry_of(task, nodes[node_idx].state);

            for (ActionIdx ai = 0; ai < task.actions.size(); ++ai) {
                const Action& action = task.actions[ai];
                if (!keep(task, stab, ai, result.stats)) continue;
                if (!action.applicable(nodes[node_idx].state)) continue;

                auto maybe = product_update(nodes[node_idx].state, action, task.frame_guard(), cap);
                if (!maybe) { result.stats.record_prune(maybe.error()); continue; }

                EpistemicState next = bisim_contract(std::move(*maybe));
                result.stats.nodes_generated++;

                const bool at_goal = next.satisfies(*task.goal);

                result.stats.heuristic_calls++;
                const float nh = at_goal ? 0.f : h(next, task);
                result.stats.final_h = nh;
                if (nh < result.stats.best_h) {
                    result.stats.best_h = nh;
                    result.stats.heuristic_improvements++;
                } else {
                    result.stats.heuristic_stalls++;
                }

                if (!at_goal && !visited.insert(next.fingerprint()).second) {
                    result.stats.duplicates_pruned++;
                    continue;
                }

                nodes.push_back(Node{std::move(next), node_idx, ai,
                                     nodes[node_idx].g + 1});
                const auto idx = static_cast<std::uint32_t>(nodes.size() - 1);

                if (at_goal) {
                    finish(idx);
                    result.stats.plateau_escapes++;
                    std::cerr << "[ehc] Solution found during BFS escape. Length="
                              << result.plan.size()
                              << "  Expanded=" << result.stats.nodes_expanded
                              << "  Generated=" << result.stats.nodes_generated << "\n";
                    return result;
                }

                if (nh < cur_h) {
                    cur_idx = idx;
                    cur_h   = nh;
                    result.stats.plateau_escapes++;
                    escaped = true;
                    break;
                }

                frontier.push(idx);
                result.stats.max_frontier_size =
                    std::max(result.stats.max_frontier_size, frontier.size());
            }
        }

        if (!escaped) {
            std::cerr << "[ehc] BFS escape exhausted — no solution.\n";
            result.stats.closed_size = visited.size();
            result.stats.stop_timer();
            return std::nullopt;
        }
    }
}

} // namespace ehc
