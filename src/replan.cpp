#include "search.hpp"

#include "bisimulation.hpp"
#include "product_update.hpp"
#include "symmetry.hpp"
#include "world_cap_policy.hpp"

#include <algorithm>
#include <deque>
#include <iostream>
#include <span>
#include <unordered_map>
#include <unordered_set>

// Conditional planning by replanning over the all-outcomes determinization.
//
// solve(s) runs GBFS from s treating every sensing outcome as a separate
// deterministic action, then commits to the first action of the path found and
// solves each of its outcome branches recursively, passing the rest of the path
// as a hint to the branch it came from. A branch that cannot be solved bans
// that action at s and GBFS runs again. Solved states are reused, so the policy
// is a DAG, and a state whose searches all fail without depending on the
// recursion stack or the deadline is recorded as dead.
//
// Sound: the policy is built from real product-update branches. Complete on
// finite tasks: a state is abandoned only after GBFS has exhausted its
// determinized reachable space under the bans.

namespace replan {
namespace {

using FingerprintSet = std::unordered_set<Fingerprint, FingerprintHash>;

struct PairHash {
    std::size_t operator()(const std::pair<Fingerprint, ActionIdx>& p) const noexcept {
        return FingerprintHash{}(p.first) ^ bits::mix64(p.second);
    }
};

struct Step {
    Fingerprint at;        // state the action is applied in
    ActionIdx   action{0};
};

struct Context {
    const PlanningTask& task;
    const Heuristic&    h;
    WorldCapPolicy      cap;
    PlannerStats&       stats;
    Deadline            deadline;

    std::unordered_map<Fingerprint, std::shared_ptr<PlanNode>, FingerprintHash> solved;
    FingerprintSet dead;
    std::unordered_set<std::pair<Fingerprint, ActionIdx>, PairHash> banned;
    FingerprintSet stack;
    bool           timed_out{false};
};

struct Result {
    bool                      ok{false};
    std::shared_ptr<PlanNode> tree;      // null = goal already holds
    bool                      tainted{false};
};

std::vector<std::pair<EventIdx, EpistemicState>>
outcomes(const EpistemicState& s, const Action& a, Context& c) {
    auto branches = product_update_split(s, a, c.task.frame_guard(), c.cap);
    for (auto& [e, b] : branches) b = bisim_contract(std::move(b));
    c.stats.nodes_generated += branches.size();
    return branches;
}

bool out_of_time(Context& c) {
    if (!expired(c.deadline)) return false;
    c.timed_out = true;
    return true;
}

// GBFS from `root` to a goal or already-solved state. Skips dead states, banned
// actions, and states on the recursion stack; `hit_stack` reports the latter,
// since a failure that depended on it is not a proof.
std::optional<std::vector<Step>>
find_path(const EpistemicState& root, const std::unordered_set<ActionIdx>& local_ban,
          Context& c, bool& hit_stack) {
    struct Node {
        CompactState   state;    // released once expanded
        Fingerprint    fp;
        std::uint32_t  parent;
        ActionIdx      action;
    };
    struct Entry {
        float         h;
        std::uint32_t g, idx;
        bool operator<(const Entry& o) const noexcept {   // max-heap on "worse"
            if (h != o.h) return h > o.h;
            return g < o.g;
        }
    };

    std::deque<Node>   nodes;
    std::vector<Entry> open;
    FingerprintSet     closed;

    const Fingerprint root_fp = root.fingerprint();
    nodes.push_back({CompactState::from(root), root_fp, UINT32_MAX, 0});
    closed.insert(root_fp);
    open.push_back({c.h(root, c.task), 0, 0});

    const auto path_to = [&](std::uint32_t i) {
        std::vector<Step> path;
        for (; nodes[i].parent != UINT32_MAX; i = nodes[i].parent)
            path.push_back({nodes[nodes[i].parent].fp, nodes[i].action});
        std::reverse(path.begin(), path.end());
        return path;
    };

    while (!open.empty()) {
        if (out_of_time(c)) return std::nullopt;

        std::pop_heap(open.begin(), open.end());
        const Entry cur = open.back();
        open.pop_back();
        c.stats.nodes_expanded++;

        const EpistemicState s  = nodes[cur.idx].state.expand();
        nodes[cur.idx].state    = CompactState{};
        const Fingerprint    fp = nodes[cur.idx].fp;
        const StateSymmetry stab = c.task.symmetry ? stabiliser(*c.task.symmetry, s)
                                                   : StateSymmetry{};

        for (ActionIdx ai = 0; ai < c.task.actions.size(); ++ai) {
            if (!stab.trivial && !stab.keep(*c.task.symmetry, ai)) continue;
            if (cur.idx == 0 && local_ban.count(ai)) continue;
            if (c.banned.count({fp, ai})) continue;
            const Action& a = c.task.actions[ai];
            if (!a.applicable(s)) continue;

            for (auto& [e, next] : outcomes(s, a, c)) {
                const Fingerprint nfp = next.fingerprint();
                if (!closed.insert(nfp).second || c.dead.count(nfp)) continue;
                if (c.stack.count(nfp)) { hit_stack = true; continue; }

                const std::uint32_t g = cur.g + 1;
                const bool done = next.satisfies(*c.task.goal) || c.solved.count(nfp);
                const float hv  = done ? 0.f : c.h(next, c.task);
                nodes.push_back({done ? CompactState{} : CompactState::from(next), nfp, cur.idx, ai});
                const auto idx = static_cast<std::uint32_t>(nodes.size() - 1);

                if (done) return path_to(idx);

                c.stats.heuristic_calls++;
                open.push_back({hv, g, idx});
                std::push_heap(open.begin(), open.end());
            }
        }
    }
    return std::nullopt;
}

Result solve(const EpistemicState& s, std::span<const Step> hint, Context& c) {
    if (out_of_time(c)) return {false, nullptr, true};

    const Fingerprint fp = s.fingerprint();
    if (s.satisfies(*c.task.goal)) return {true, nullptr, false};
    if (auto it = c.solved.find(fp); it != c.solved.end()) return {true, it->second, false};
    if (c.dead.count(fp))  return {false, nullptr, false};
    if (c.stack.count(fp)) return {false, nullptr, true};

    c.stack.insert(fp);
    std::unordered_set<ActionIdx> local_ban;
    bool tainted = false;

    for (bool first = true;; first = false) {
        ActionIdx               action;
        std::vector<Step>       path;
        std::span<const Step>   rest;

        if (first && !hint.empty() && hint.front().at == fp) {
            action = hint.front().action;
            rest   = hint.subspan(1);
        } else {
            bool hit_stack = false;
            auto found = find_path(s, local_ban, c, hit_stack);
            if (!found) {
                c.stack.erase(fp);
                tainted |= hit_stack || c.timed_out;
                if (!tainted) c.dead.insert(fp);
                return {false, nullptr, tainted};
            }
            path   = std::move(*found);
            action = path.front().action;
            rest   = std::span<const Step>(path).subspan(1);
        }

        auto node = std::make_shared<PlanNode>();
        node->action = c.task.actions[action].name;

        bool ok = true;
        for (auto& [e, branch] : outcomes(s, c.task.actions[action], c)) {
            const std::span<const Step> sub =
                (!rest.empty() && rest.front().at == branch.fingerprint()) ? rest
                                                                           : std::span<const Step>{};
            const Result r = solve(branch, sub, c);
            if (!r.ok) {
                ok = false;
                if (r.tainted) { tainted = true; local_ban.insert(action); }
                else           c.banned.insert({fp, action});
                break;
            }
            node->branches.emplace_back(e, r.tree);
        }

        if (ok && !node->branches.empty()) {
            c.stack.erase(fp);
            c.solved[fp] = node;
            return {true, node, false};
        }
        if (ok) c.banned.insert({fp, action});   // inapplicable hint
    }
}

} // namespace

std::optional<ConditionalSearchResult>
search(const PlanningTask& task, const Heuristic& h, Deadline deadline) {
    ConditionalSearchResult out;
    out.stats.start_timer();

    const EpistemicState init = bisim_contract(task.init);
    Context c{task, h, make_world_cap_policy(task.partial_obs), out.stats, deadline,
              {}, {}, {}, {}, false};

    const Result r = solve(init, {}, c);
    out.stats.stop_timer();

    if (!r.ok) {
        std::cerr << (c.timed_out ? "[replan] Deadline exceeded." : "[replan] No solution exists.")
                  << "  Expanded=" << out.stats.nodes_expanded
                  << "  Generated=" << out.stats.nodes_generated
                  << "  Solved=" << c.solved.size() << "  Dead=" << c.dead.size() << "\n";
        return std::nullopt;
    }
    out.plan_tree = r.tree;
    std::cerr << "[replan] Solution found  Expanded=" << out.stats.nodes_expanded
              << "  Generated=" << out.stats.nodes_generated
              << "  Solved=" << c.solved.size() << "  Dead=" << c.dead.size()
              << "  Banned=" << c.banned.size() << "\n";
    return out;
}

} // namespace replan
