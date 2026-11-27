#include "symsearch.hpp"
#include <utility>

#include "bisimulation.hpp"
#include "hdelta.hpp"
#include "knowledge_relaxation.hpp"
#include "parallel.hpp"
#include "product_update.hpp"
#include "symdel.hpp"
#include "symheur.hpp"
#include "symmetry.hpp"
#include "world_cap_policy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace symsearch {

bool supports(const std::string& h) {
    return h == "kadd" || h == "kff" || h == "hd" || h == "hdsum" || h == "ug";
}

namespace {

using Clock = std::chrono::steady_clock;
constexpr std::uint32_t kNoNode = std::numeric_limits<std::uint32_t>::max();

struct Node {
    bool           symbolic{false};
    bool           live{false};        // state held (not yet expanded)
    CompactState   ex;
    sym::SymState  sy;
    std::uint32_t  parent{kNoNode};
    ActionIdx      action{0};
    std::uint32_t  g{0};
    double         worlds{0};
    // Swaps known to fix this (symbolic) state, by index; empty = unknown.
    std::vector<std::uint8_t> fixing;
};

struct QEntry { float h; std::uint32_t g; std::uint32_t idx; };

// What a worker computes for one explicit successor; the merge only reads it.
struct ExSucc {
    bool           applicable{false};
    PruneReason    pruned{PruneReason::None};
    bool           symbolic{false};    // goes to the BDD side: nothing else computed
    bool           goal{false};
    bool           known{false};       // closed when built: h not computed
    bool           dead{false};
    float          h{0.f};
    Fingerprint    fp;
    EpistemicState state;
};

// Every extension successor generation reads, so that concurrent sat() calls
// on the parent only look up (as in gbfs).
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
struct Worse {
    bool operator()(const QEntry& a, const QEntry& b) const noexcept {
        if (a.h != b.h) return a.h > b.h;
        return a.g < b.g;
    }
};

struct Timer {
    double& acc;
    Clock::time_point t0 = Clock::now();
    ~Timer() { acc += std::chrono::duration<double>(Clock::now() - t0).count(); }
};

class Search {
public:
    Search(const PlanningTask& t, const Heuristic& h, const Options& o, Stats& st)
        : t_(t), h_(h), o_(o), st_(st), eng_(t),
          kr_(dynamic_cast<const KnowledgeRelaxationHeuristic*>(&h)),
          cap_(make_world_cap_policy(t.partial_obs)) {
        if ((o.heuristic == "kadd" || o.heuristic == "kff") && !kr_)
            throw std::logic_error("symsearch: kadd/kff without a knowledge-relaxation heuristic");
    }

    std::optional<SearchResult> run(bool helpful, bool& pruned, bool root_check);

private:
    const PlanningTask& t_;
    const Heuristic&    h_;
    const Options&      o_;
    Stats&              st_;
    sym::Engine         eng_;
    const KnowledgeRelaxationHeuristic* kr_;
    WorldCapPolicy      cap_;
    AgentSymmetry       sym_;
    bool                use_sym_{false};
    double              th_{0}, thh_{0};      // thresholds of this run

    std::deque<Node>    nodes_;
    std::vector<QEntry> open_;
    std::unordered_set<Fingerprint, FingerprintHash> closed_;
    std::unordered_map<Fingerprint, std::vector<std::uint32_t>, FingerprintHash> hidden_by_inv_;
    std::vector<sym::SymState> hidden_;
    SearchResult        res_;
    std::vector<std::uint8_t> pending_fix_;   // for the next symbolic node created

    float hsym(sym::SymState& s);
    bool  dead_sym(sym::SymState& s, float h);
    void  maybe_gc();
    // A successor in either representation; returns true when it is the goal.
    bool  add_explicit(EpistemicState e, std::uint32_t parent, ActionIdx a, std::uint32_t g,
                       const ExSucc* pre = nullptr);
    bool  explicit_side(const EpistemicState& e) const;
    bool  add_symbolic(sym::SymState s, std::uint32_t parent, ActionIdx a, std::uint32_t g);
    bool  add(EpistemicState e, std::uint32_t parent, ActionIdx a, std::uint32_t g);
    bool  add(sym::SymState s, std::uint32_t parent, ActionIdx a, std::uint32_t g);
    void  push(Node n, float h);
    std::vector<std::string> plan(std::uint32_t leaf) const;
    void  finish();
};

float Search::hsym(sym::SymState& s) {
    Timer tm{st_.t_heuristic};
    const std::string& l = o_.heuristic;
    if (l == "kadd" || l == "kff") return kr_->estimate(sym::initial_facts(eng_, s, *kr_));
    if (l == "hd") {
        const auto v = sym::h_delta(eng_, t_, s);
        return v == sym::Relaxation::kInf ? std::numeric_limits<float>::infinity() : float(v);
    }
    if (l == "hdsum") {
        const auto v = sym::h_delta_both(eng_, t_, s).sum;
        return v == sym::Relaxation::kInf ? std::numeric_limits<float>::infinity() : float(v);
    }
    // ug: unsatisfied top-level goal conjuncts
    float n = 0;
    if (t_.goal->kind == FormulaKind::And) {
        for (const auto& c : t_.goal->children) n += !eng_.holds(s, *c);
    } else {
        n = !eng_.holds(s, *t_.goal);
    }
    return n;
}

bool Search::dead_sym(sym::SymState& s, float h) {
    switch (t_.dead_end_check) {
    case DeadEndCheck::Off:
    case DeadEndCheck::Root:
        return false;
    case DeadEndCheck::Suspect: {
        if (std::isinf(h)) return true;
        const int dead = std::exchange(last_dead_conjuncts, -1);
        if (dead >= 0 ? dead == 0 : h < t_.dead_end_suspect) return false;
        break;
    }
    case DeadEndCheck::All:
        if (std::isinf(h)) return true;
        break;
    }
    Timer tm{st_.t_heuristic};
    return sym::h_delta(eng_, t_, s) == sym::Relaxation::kInf;
}

void Search::maybe_gc() {
    st_.peak_bdd_nodes = std::max(st_.peak_bdd_nodes, eng_.mgr().live());
    st_.peak_bdd_bytes = std::max(st_.peak_bdd_bytes, eng_.mgr().bytes());
    if (!eng_.mgr().should_gc()) return;
    std::vector<bdd::Ref> roots;
    for (const Node& n : nodes_) if (n.symbolic && n.live) sym::Engine::roots_of(n.sy, roots);
    for (const auto& s : hidden_) sym::Engine::roots_of(s, roots);
    eng_.gc(std::move(roots));
    ++st_.gc_runs;
}

void Search::push(Node n, float h) {
    const std::uint32_t g = n.g;
    nodes_.push_back(std::move(n));
    open_.push_back(QEntry{h, g, static_cast<std::uint32_t>(nodes_.size() - 1)});
    std::push_heap(open_.begin(), open_.end(), Worse{});
    res_.stats.max_frontier_size = std::max(res_.stats.max_frontier_size, open_.size());
    res_.stats.final_h = h;
    if (h < res_.stats.best_h) { res_.stats.best_h = h; ++res_.stats.heuristic_improvements; }
    else ++res_.stats.heuristic_stalls;
}

// Valuation-injective: no two worlds share a valuation. For a contracted
// state this is a bisimulation invariant, and it is exactly H = ∅ in the
// symbolic encoding.
bool injective(const EpistemicState& e) {
    std::unordered_map<std::uint64_t, std::vector<WorldIdx>> by_hash;
    for (WorldIdx w = 0; w < e.num_worlds; ++w) {
        const auto v = e.val(w);
        std::uint64_t h = 0x9E3779B97F4A7C15ull;
        for (bits::Word x : v) h = bits::mix64(h ^ x);
        auto& c = by_hash[h];
        for (WorldIdx u : c) {
            const auto o = e.val(u);
            if (std::equal(v.begin(), v.end(), o.begin(), o.end())) return false;
        }
        c.push_back(w);
    }
    return true;
}

bool Search::explicit_side(const EpistemicState& e) const {
    const double w = double(e.num_worlds);
    return w <= th_ || (w <= thh_ && !injective(e));
}

bool Search::add(EpistemicState e, std::uint32_t parent, ActionIdx a, std::uint32_t g) {
    if (explicit_side(e)) return add_explicit(std::move(e), parent, a, g);
    ++st_.to_symbolic;
    sym::SymState s;
    {
        Timer tm{st_.t_convert};
        s = eng_.contract(eng_.from_explicit(e));
    }
    return add_symbolic(std::move(s), parent, a, g);
}

bool Search::add(sym::SymState s, std::uint32_t parent, ActionIdx a, std::uint32_t g) {
    double w;
    {
        Timer tm{st_.t_identity};
        w = eng_.worlds(s);
    }
    if (w > th_ && (s.hid.empty() || w > thh_))
        return add_symbolic(std::move(s), parent, a, g);
    ++st_.to_explicit;
    EpistemicState e;
    {
        Timer tm{st_.t_convert};
        e = bisim_contract(eng_.explicit_state(s));
    }
    return add_explicit(std::move(e), parent, a, g);
}

bool Search::add_explicit(EpistemicState e, std::uint32_t parent, ActionIdx a, std::uint32_t g,
                          const ExSucc* pre) {
    ++st_.explicit_generated;
    ++res_.stats.nodes_generated;
    st_.max_worlds = std::max(st_.max_worlds, double(e.num_worlds));
    if (pre ? pre->goal : e.satisfies(*t_.goal)) {
        nodes_.push_back(Node{false, false, {}, {}, parent, a, g, double(e.num_worlds)});
        return true;
    }
    if (!closed_.insert(pre ? pre->fp : e.fingerprint()).second) { ++res_.stats.duplicates_pruned; return false; }
    ++res_.stats.heuristic_calls;
    float hv;
    if (pre && !pre->known) {
        hv = pre->h;
        if (pre->dead) { res_.stats.record_prune(PruneReason::DeadEnd); return false; }
    } else {
        Timer tm{st_.t_heuristic};
        hv = h_(e, t_);
        if (hdelta::prunes(t_, e, hv)) { res_.stats.record_prune(PruneReason::DeadEnd); return false; }
    }
    Node n{false, true, CompactState::from(e), {}, parent, a, g, double(e.num_worlds)};
    push(std::move(n), hv);
    return false;
}

bool Search::add_symbolic(sym::SymState s, std::uint32_t parent, ActionIdx a, std::uint32_t g) {
    ++st_.symbolic_generated;
    ++res_.stats.nodes_generated;
    const double w = eng_.worlds(s);
    st_.max_worlds = std::max(st_.max_worlds, w);
    bool at_goal;
    {
        Timer tm{st_.t_goal};
        at_goal = eng_.holds(s, *t_.goal);
    }
    if (at_goal) {
        nodes_.push_back(Node{true, false, {}, {}, parent, a, g, w});
        return true;
    }
    {
        Timer tm{st_.t_identity};
        const Fingerprint fp = eng_.fingerprint(s);
        if (!s.hid.empty()) ++st_.hidden_generated;
        if (closed_.contains(fp)) {
            if (!s.hid.empty()) ++st_.hidden_same_encoding;
            ++res_.stats.duplicates_pruned;
            return false;
        }
        if (!s.hid.empty()) {
            auto& bucket = hidden_by_inv_[eng_.invariant(s)];
            for (std::uint32_t k : bucket) {
                ++st_.hidden_checks;
                if (eng_.equivalent(s, hidden_[k])) {
                    ++st_.hidden_bisimilar;
                    ++res_.stats.duplicates_pruned;
                    closed_.insert(fp);
                    return false;
                }
            }
            sym::SymState keep = s;
            keep.memo.clear();
            bucket.push_back(static_cast<std::uint32_t>(hidden_.size()));
            hidden_.push_back(std::move(keep));
            st_.hidden_closed = hidden_.size();
        }
        closed_.insert(fp);
    }
    ++res_.stats.heuristic_calls;
    const float hv = hsym(s);
    if (dead_sym(s, hv)) { res_.stats.record_prune(PruneReason::DeadEnd); return false; }
    s.memo.clear();
    Node n{true, true, {}, std::move(s), parent, a, g, w};
    n.fixing = std::move(pending_fix_);
    pending_fix_.clear();
    push(std::move(n), hv);
    return false;
}

std::vector<std::string> Search::plan(std::uint32_t leaf) const {
    std::vector<std::string> p;
    for (std::uint32_t i = leaf; i != kNoNode && nodes_[i].parent != kNoNode; i = nodes_[i].parent)
        p.push_back(t_.actions[nodes_[i].action].name);
    std::reverse(p.begin(), p.end());
    return p;
}

void Search::finish() {
    const auto& es = eng_.stats;
    std::cerr << "[symsearch] swaps inherited " << st_.symmetric_inherited << ", tested " << st_.symmetric_tests << " (H != 0: same BDDs " << es.fixes_same << ", hidden relabelling " << es.fixes_relabel
              << ", invariant differs " << es.fixes_invariant << ", bisimilar " << es.fixes_equivalent
              << ", not bisimilar " << es.fixes_rejected << ")\n";
    st_.contractions = es.contractions;
    st_.fixpoint_skipped = es.fixpoint_skipped;
    st_.fixpoint_runs = es.fixpoint_runs;
    st_.fixpoint_rounds = es.fixpoint_rounds;
    st_.peak_bdd_nodes = std::max(st_.peak_bdd_nodes, eng_.mgr().peak_live());
    res_.stats.closed_size = closed_.size();
    res_.stats.stop_timer();
}

std::optional<SearchResult> Search::run(bool helpful, bool& pruned, bool root_check) {
    res_ = SearchResult{};
    res_.stats.start_timer();
    nodes_.clear(); open_.clear(); closed_.clear(); hidden_by_inv_.clear(); hidden_.clear();

    // Root, in the representation its size calls for.
    const bool from_theory = t_.theory && t_.init.num_worlds == 0;
    if (!from_theory && t_.init.num_worlds == 0) throw std::runtime_error("symsearch: empty initial state");

    // Swaps were verified on task.init; when that is not materialised, verify
    // them again on the symbolic initial state.
    if (t_.symmetry) {
        sym_ = *t_.symmetry;
        if (from_theory) {
            sym::SymState s0 = eng_.contract(eng_.from_theory(*t_.theory));
            std::vector<AgentSymmetry::Swap> kept;
            for (const auto& sw : sym_.swaps) if (eng_.fixes(s0, sw)) kept.push_back(sw);
            if (kept.size() != sym_.swaps.size())
                std::cerr << "[symsearch] " << sym_.swaps.size() - kept.size()
                          << " agent swaps do not fix the initial state and are dropped\n";
            sym_.swaps = std::move(kept);
        }
        use_sym_ = !sym_.swaps.empty();
    }

    // Dead-end test at the root, with at most a tenth of the remaining time.
    auto root_dead = [&](auto&& test) {
        if (!root_check || t_.dead_end_check == DeadEndCheck::Off) return false;
        const auto now = Clock::now();
        const auto budget = o_.deadline == Deadline::max() ? std::chrono::seconds(30)
            : std::chrono::duration_cast<Clock::duration>((o_.deadline - now) / 10);
        const auto r = test(std::min(o_.deadline, now + budget));
        if (r) std::cerr << "[hdelta] h(s0) = " << (*r == hdelta::kInf ? std::string("inf") : std::to_string(*r)) << "\n";
        return r && *r == hdelta::kInf;
    };

    th_ = o_.threshold;
    thh_ = std::max(o_.threshold, o_.hidden_threshold);
    if (o_.full_from > 0) {
        const double w0 = from_theory ? o_.full_from : double(bisim_contract(t_.init).num_worlds);
        if (w0 >= o_.full_from) { th_ = thh_ = 0; st_.full_symbolic = true; }
    }
    if (st_.full_symbolic)
        std::cerr << "[symsearch] initial state has at least " << o_.full_from << " worlds: every state symbolic\n";

    bool goal = false;
    if (from_theory || (double(t_.init.num_worlds) > th_ &&
                        (double(t_.init.num_worlds) > thh_ || injective(bisim_contract(t_.init))))) {
        sym::SymState s0 = eng_.contract(t_.theory ? eng_.from_theory(*t_.theory) : eng_.from_explicit(t_.init));
        std::cerr << "[symsearch] initial state: " << eng_.worlds(s0) << " worlds (symbolic, "
                  << eng_.nodes(s0) << " BDD nodes)\n";
        if (root_dead([&](auto until) { return sym::h_delta_until(eng_, t_, s0, until); })) {
            std::cerr << "[hdelta] No solution exists: the relaxation proves the goal unreachable.\n";
            finish();
            return std::nullopt;
        }
        goal = add(std::move(s0), kNoNode, 0, 0);
    } else {
        EpistemicState e0 = bisim_contract(t_.init);
        std::cerr << "[symsearch] initial state: " << e0.num_worlds << " worlds\n";
        if (root_dead([&](auto until) { return hdelta::h_delta_until(t_, e0, until); })) {
            std::cerr << "[hdelta] No solution exists: the relaxation proves the goal unreachable.\n";
            finish();
            return std::nullopt;
        }
        goal = add(std::move(e0), kNoNode, 0, 0);
    }
    --res_.stats.nodes_generated;
    if (goal) { finish(); return res_; }
    if (!open_.empty()) res_.stats.initial_h = res_.stats.best_h = open_.front().h;

    std::vector<ActionIdx> pref, cands;
    while (!open_.empty()) {
        std::pop_heap(open_.begin(), open_.end(), Worse{});
        const QEntry cur = open_.back();
        open_.pop_back();
        ++res_.stats.nodes_expanded;
        if (o_.max_nodes > 0 && res_.stats.nodes_expanded > o_.max_nodes) {
            std::cerr << "[symsearch] Node limit reached (" << o_.max_nodes << ").\n";
            finish();
            return std::nullopt;
        }
        if (expired(o_.deadline)) {
            std::cerr << "[symsearch] Deadline exceeded at " << res_.stats.nodes_expanded << " nodes.\n";
            finish();
            return std::nullopt;
        }
        const std::uint32_t ci = cur.idx;
        const std::uint32_t g = nodes_[ci].g + 1;
        bool any = false;

        if (!nodes_[ci].symbolic) {
            const EpistemicState P = nodes_[ci].ex.expand();
            nodes_[ci].ex = CompactState{};
            nodes_[ci].live = false;
            StateSymmetry stab;
            if (use_sym_) {
                Timer tm{st_.t_symmetry};
                stab = stabiliser(sym_, P);
            }
            cands.clear();
            for (ActionIdx ai = 0; ai < t_.actions.size(); ++ai)
                if (stab.trivial || stab.keep(sym_, ai)) cands.push_back(ai);
                else ++res_.stats.pruned_symmetric;
            bool has_pref = false;
            if (helpful) {
                Timer tm{st_.t_preferred};
                has_pref = h_.preferred(P, t_, pref);
            }
            if (has_pref) {
                std::vector<ActionIdx> kept;
                for (ActionIdx ai : cands) if (std::binary_search(pref.begin(), pref.end(), ai)) kept.push_back(ai);
                if (!kept.empty() && kept.size() < cands.size()) { cands.swap(kept); pruned = true; }
            }
            // Successors are built independently, in parallel when the model
            // is large enough to pay for it, and merged in action order.
            std::vector<ExSucc> succ(cands.size());
            const auto build = [&](std::size_t k) {
                ExSucc& sc = succ[k];
                const Action& act = t_.actions[cands[k]];
                if (!act.applicable(P)) return;
                sc.applicable = true;
                auto r = product_update(P, act, t_.seriality(), cap_);
                if (!r) { sc.pruned = r.error(); return; }
                sc.state = bisim_contract(std::move(*r));
                if (!explicit_side(sc.state)) { sc.symbolic = true; return; }
                sc.fp = sc.state.fingerprint();
                sc.goal = sc.state.satisfies(*t_.goal);
                sc.known = closed_.contains(sc.fp);
                if (!sc.goal && !sc.known) {
                    sc.h = h_(sc.state, t_);
                    sc.dead = hdelta::prunes(t_, sc.state, sc.h);
                }
            };
            {
                Timer tm{st_.t_update};
                if (par::threads() > 1 && cands.size() >= 4 &&
                    std::size_t(P.num_worlds) * cands.size() >= 4096) {
                    warm_extensions(P, t_);
                    par::for_each_index(cands.size(), build);
                } else {
                    for (std::size_t k = 0; k < cands.size(); ++k) build(k);
                }
            }
            for (std::size_t k = 0; k < cands.size(); ++k) {
                ExSucc& sc = succ[k];
                if (!sc.applicable) continue;
                if (sc.pruned != PruneReason::None) { res_.stats.record_prune(sc.pruned); continue; }
                any = true;
                bool hit;
                if (sc.symbolic) {
                    hit = add(std::move(sc.state), ci, cands[k], g);
                } else {
                    hit = add_explicit(std::move(sc.state), ci, cands[k], g, &sc);
                }
                if (hit) {
                    res_.plan = plan(static_cast<std::uint32_t>(nodes_.size() - 1));
                    finish();
                    return res_;
                }
            }
        } else {
            ++st_.symbolic_expanded;
            sym::SymState P = std::move(nodes_[ci].sy);
            nodes_[ci].sy = sym::SymState{};
            nodes_[ci].live = false;
            StateSymmetry stab;
            // Swaps proved to fix P. A swap that fixes P and maps action a to
            // itself fixes P ⊗ a (renaming commutes with product update, which
            // preserves bisimilarity), so successors inherit them untested.
            std::vector<std::uint8_t> fixed;
            if (use_sym_) {
                Timer tm{st_.t_symmetry};
                const std::vector<std::uint8_t> known = std::move(nodes_[ci].fixing);
                nodes_[ci].fixing.clear();
                fixed.assign(sym_.swaps.size(), 0);
                std::optional<sym::Engine::SwapFilter> filter;
                stab = stabiliser_by(sym_, static_cast<std::uint32_t>(t_.num_agents()),
                                     [&](const AgentSymmetry::Swap& sw) {
                                         const std::size_t k = std::size_t(&sw - sym_.swaps.data());
                                         if (!filter) filter = eng_.swap_filter(P);
                                         if (!filter->admits(sw)) return false;
                                         ++st_.symmetric_tests;
                                         return bool(fixed[k] = eng_.fixes(P, sw));
                                     }, known.empty() ? nullptr : &known);
                for (std::size_t k = 0; k < known.size(); ++k)
                    if (known[k]) { fixed[k] = 1; ++st_.symmetric_inherited; }
            }
            cands.clear();
            for (ActionIdx ai = 0; ai < t_.actions.size(); ++ai)
                if (stab.trivial || stab.keep(sym_, ai)) cands.push_back(ai);
                else ++res_.stats.pruned_symmetric;
            if (helpful && kr_) {
                Timer tm{st_.t_preferred};
                kr_->preferred_from(sym::initial_facts(eng_, P, *kr_), pref);
                std::vector<ActionIdx> kept;
                for (ActionIdx ai : cands) if (std::binary_search(pref.begin(), pref.end(), ai)) kept.push_back(ai);
                if (!kept.empty() && kept.size() < cands.size()) { cands.swap(kept); pruned = true; }
            }
            for (ActionIdx ai : cands) {
                const Action& act = t_.actions[ai];
                bool ok;
                {
                    Timer tm{st_.t_goal};
                    ok = eng_.applicable(P, act);
                }
                if (!ok) continue;
                if (use_sym_) {
                    pending_fix_.assign(sym_.swaps.size(), 0);
                    for (std::size_t k = 0; k < sym_.swaps.size(); ++k)
                        pending_fix_[k] = fixed[k] && sym_.swaps[k].action[ai] == ai;
                }
                sym::SymState u;
                {
                    Timer tm{st_.t_update};
                    u = eng_.update(P, act);
                }
                {
                    Timer tm{st_.t_contract};
                    u = eng_.contract(u);
                }
                any = true;
                const bool hit = add(std::move(u), ci, ai, g);
                pending_fix_.clear();
                if (hit) {
                    res_.plan = plan(static_cast<std::uint32_t>(nodes_.size() - 1));
                    finish();
                    return res_;
                }
            }
        }
        if (!any) ++res_.stats.dead_ends;
        maybe_gc();
    }
    std::cerr << (pruned ? "[symsearch] Helpful-action search exhausted.\n"
                         : "[symsearch] Search exhausted — no solution.\n");
    finish();
    return std::nullopt;
}

} // namespace

std::optional<SearchResult> search(const PlanningTask& task, const Heuristic& h,
                                   const Options& opt, Stats* stats) {
    if (task.seriality() != Seriality::Ignore)
        throw std::runtime_error("symbolic search: seriality repair and --consistent-beliefs are not supported");
    if (!supports(opt.heuristic))
        throw std::runtime_error("symbolic search: heuristic '" + opt.heuristic +
                                 "' has no symbolic evaluation (use kadd, kff, hd, hdsum or ug)");
    Stats local;
    Stats& st = stats ? *stats : local;
    bool pruned = false;
    std::optional<SearchResult> r;
    {
        Search s(task, h, opt, st);
        r = s.run(opt.helpful, pruned, true);
    }
    if (r || !pruned || expired(opt.deadline)) return r;
    std::cerr << "[symsearch] Retrying without helpful-action pruning.\n";
    st = Stats{};
    Search s(task, h, opt, st);
    return s.run(false, pruned, false);
}

} // namespace symsearch
