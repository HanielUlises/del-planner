// Tests for the symbolic search layer: BDD garbage collection and adjacent
// swaps, state identity across encodings, the symbolic heuristics against the
// explicit ones, the symbolic symmetry test, and hybrid search end to end.
//
// States come from breadth-first expansion with the explicit planner
// (product_update + bisim_contract), whose canonical fingerprint is the
// reference for identity.
#include "../temporal/check.hpp"

#include "bisimulation.hpp"
#include "hdelta.hpp"
#include "knowledge_relaxation.hpp"
#include "parser.hpp"
#include "product_update.hpp"
#include "symdel.hpp"
#include "symheur.hpp"
#include "symmetry.hpp"
#include "symsearch.hpp"

#include <deque>
#include <iostream>
#include <random>
#include <unordered_map>

namespace {

const std::string kRoot = SYMBOLIC_SOURCE_DIR;

PlanningTask load(const std::string& rel) {
    std::cerr.setstate(std::ios::failbit);
    PlanningTask t = load_task(kRoot + "/" + rel);
    std::cerr.clear();
    return t;
}

const std::vector<std::string>& fixtures() {
    static const std::vector<std::string> f{
        "benchmarks/gossip1/problem_1.json",
        "benchmarks/coin1/problem_1.json",
        "benchmarks/coin3/problem_3.json",
        "benchmarks/muddy1/muddy-children-problem.json",
        "benchmarks/sally-anne/problem_1.json",
        "benchmarks/grapevine1/problem_1.json",
        "benchmarks/whisper/whisper-problem.json",
        "benchmarks/amc1/amc-problem.json",
        "tests/symbolic/intermediate-gos-05-all.json",
        "tests/symbolic/hard-gos-04-imp-deceived.json",
    };
    return f;
}

// A reachable state with the action that produced it from its parent.
struct Reached {
    EpistemicState state;
    int            parent{-1};
    ActionIdx      action{0};
};

std::vector<Reached> explore(const PlanningTask& t, std::size_t limit) {
    std::vector<Reached> out;
    std::unordered_map<Fingerprint, int, FingerprintHash> seen;
    out.push_back({bisim_contract(t.init), -1, 0});
    seen.emplace(out[0].state.fingerprint(), 0);
    for (std::size_t i = 0; i < out.size() && out.size() < limit; ++i) {
        for (ActionIdx a = 0; a < t.actions.size() && out.size() < limit; ++a) {
            const EpistemicState p = out[i].state;
            if (!t.actions[a].applicable(p)) continue;
            auto r = product_update(p, t.actions[a], Seriality::Ignore);
            if (!r) continue;
            EpistemicState e = bisim_contract(std::move(*r));
            if (seen.emplace(e.fingerprint(), int(out.size())).second)
                out.push_back({std::move(e), int(i), a});
        }
    }
    return out;
}

// --------------------------------------------------------------------- BDD

TEST(gc_keeps_roots_and_reuses_freed_nodes) {
    bdd::Manager m(24);
    std::mt19937 rng(7);
    auto random_fn = [&](int clauses) {
        bdd::Ref f = bdd::kFalse;
        for (int c = 0; c < clauses; ++c) {
            bdd::Ref cl = bdd::kTrue;
            for (int k = 0; k < 4; ++k) {
                const std::uint32_t v = rng() % 24;
                cl = m.band(cl, rng() % 2 ? m.var(v) : m.nvar(v));
            }
            f = m.bor(f, cl);
        }
        return f;
    };
    std::vector<std::uint32_t> all(24);
    for (std::uint32_t v = 0; v < 24; ++v) all[v] = v;
    std::vector<bdd::Ref> keep;
    std::vector<double> counts;
    for (int i = 0; i < 20; ++i) {
        const bdd::Ref f = random_fn(30);
        (void)random_fn(30);                     // garbage
        keep.push_back(f);
        counts.push_back(m.count(f, all));
    }
    const std::size_t before = m.live();
    const std::size_t freed = m.gc(keep);
    CHECK(freed > 0);
    CHECK(m.live() == before - freed);
    for (std::size_t i = 0; i < keep.size(); ++i) CHECK(m.count(keep[i], all) == counts[i]);
    // Rebuilding a kept function finds the same node; new work reuses slots.
    const std::size_t slots = m.size();
    for (int i = 0; i < 5; ++i) (void)random_fn(30);
    CHECK(m.size() <= slots + 1);
    // Operations on kept nodes after a collection agree with a fresh manager.
    for (std::size_t i = 0; i + 1 < keep.size(); ++i) {
        const bdd::Ref g = m.band(keep[i], m.neg(keep[i + 1]));
        CHECK(m.count(g, all) == m.count(keep[i], all) - m.count(m.band(keep[i], keep[i + 1]), all));
    }
}

TEST(swap_adjacent_matches_permute) {
    bdd::Manager m(30);
    std::mt19937 rng(11);
    std::vector<std::uint8_t> first(30, 0);
    std::vector<std::uint32_t> perm(30);
    for (std::uint32_t v = 0; v < 30; ++v) perm[v] = v;
    for (std::uint32_t v = 0; v + 1 < 30; v += 3) { first[v] = 1; perm[v] = v + 1; perm[v + 1] = v; }
    for (int i = 0; i < 50; ++i) {
        bdd::Ref f = bdd::kFalse;
        for (int c = 0; c < 12; ++c) {
            bdd::Ref cl = bdd::kTrue;
            for (int k = 0; k < 5; ++k) {
                const std::uint32_t v = rng() % 30;
                cl = m.band(cl, rng() % 2 ? m.var(v) : m.nvar(v));
            }
            f = m.bor(f, cl);
        }
        CHECK(m.swap_adjacent(f, first) == m.permute(f, perm, false));
    }
}

// ----------------------------------------------------------------- identity

// Symbolic successors equal the encoding of the explicit successors: the same
// fingerprint when H = ∅, otherwise the same invariant and equivalent().
// Distinct explicit states are never equivalent symbolically.
TEST(identity_agrees_with_explicit_canonical_form) {
    std::size_t states = 0, hidden = 0, pairs = 0;
    for (const auto& f : fixtures()) {
        const PlanningTask t = load(f);
        const auto R = explore(t, 60);
        sym::Engine eng(t);
        std::vector<sym::SymState> S;
        for (const auto& r : R) S.push_back(eng.contract(eng.from_explicit(r.state)));
        for (std::size_t i = 1; i < R.size(); ++i) {
            sym::SymState p = S[R[i].parent];
            sym::SymState u = eng.contract(eng.update(p, t.actions[R[i].action]));
            ++states;
            CHECK_MSG(eng.worlds(u) == double(R[i].state.num_worlds), f);
            CHECK_MSG(bisim_contract(eng.explicit_state(u)).fingerprint() == R[i].state.fingerprint(), f);
            if (u.hid.empty() && S[i].hid.empty()) {
                CHECK_MSG(eng.fingerprint(u) == eng.fingerprint(S[i]), f);
            } else {
                ++hidden;
                CHECK_MSG(eng.invariant(u) == eng.invariant(S[i]), f);
                CHECK_MSG(eng.equivalent(u, S[i]), f);
            }
        }
        // Distinct states: never equivalent (spot-check neighbours).
        for (std::size_t i = 0; i + 1 < S.size(); ++i) {
            ++pairs;
            CHECK_MSG(!eng.equivalent(S[i], S[i + 1]), f);
            if (S[i].hid.empty() && S[i + 1].hid.empty())
                CHECK_MSG(eng.fingerprint(S[i]) != eng.fingerprint(S[i + 1]), f);
        }
    }
    std::cout << "  " << states << " successors (" << hidden << " with H != 0), "
              << pairs << " distinct pairs\n";
    CHECK(hidden > 0);
}

// Collecting with only some states as roots leaves them intact.
TEST(gc_preserves_rooted_states) {
    const PlanningTask t = load("tests/symbolic/intermediate-gos-05-all.json");
    const auto R = explore(t, 40);
    sym::Engine eng(t);
    std::vector<sym::SymState> S;
    std::vector<bdd::Ref> roots;
    for (std::size_t i = 0; i < R.size(); ++i) {
        sym::SymState s = eng.contract(eng.from_explicit(R[i].state));
        (void)eng.holds(s, *t.goal);              // fill the memo, then drop it
        s.memo.clear();
        if (i % 2 == 0) { sym::Engine::roots_of(s, roots); S.push_back(std::move(s)); }
    }
    CHECK(eng.gc(roots) > 0);
    for (std::size_t i = 0; i < S.size(); ++i)
        CHECK(bisim_contract(eng.explicit_state(S[i])).fingerprint() == R[2 * i].state.fingerprint());
}

// --------------------------------------------------------------- heuristics

TEST(symbolic_kadd_equals_explicit_kadd) {
    std::size_t n = 0;
    for (const auto& f : fixtures()) {
        const PlanningTask t = load(f);
        const auto R = explore(t, 40);
        sym::Engine eng(t);
        for (auto est : {KnowledgeRelaxationHeuristic::Estimate::Add, KnowledgeRelaxationHeuristic::Estimate::FF}) {
            const KnowledgeRelaxationHeuristic h(t, est);
            for (const auto& r : R) {
                sym::SymState s = eng.contract(eng.from_explicit(r.state));
                CHECK_MSG(h.initial_facts(r.state) == sym::initial_facts(eng, s, h), f);
                CHECK_MSG(h(r.state, t) == h.estimate(sym::initial_facts(eng, s, h)), f);
                ++n;
            }
        }
    }
    std::cout << "  " << n << " evaluations\n";
}

// Without R1–R4 the symbolic relaxation over-approximates the explicit one:
// its h^Δ is never larger, and ∞ symbolically implies ∞ explicitly.
TEST(symbolic_hdelta_is_below_explicit_hdelta) {
    std::size_t n = 0, equal = 0;
    for (const auto& f : fixtures()) {
        const PlanningTask t = load(f);
        const auto R = explore(t, 25);
        sym::Engine eng(t);
        for (const auto& r : R) {
            sym::SymState s = eng.contract(eng.from_explicit(r.state));
            const auto hs = sym::h_delta(eng, t, s);
            const auto he = hdelta::h_delta(t, r.state);
            CHECK_MSG(hs <= he, f + ": " + std::to_string(hs) + " > " + std::to_string(he));
            ++n;
            equal += hs == he;
        }
    }
    std::cout << "  " << n << " states, " << equal << " with equal values\n";
}

// ----------------------------------------------------------------- symmetry

TEST(symbolic_swap_test_matches_explicit_stabiliser) {
    std::size_t n = 0, fixed = 0;
    for (const char* f : {"benchmarks/gossip1/problem_1.json", "tests/symbolic/intermediate-gos-05-all.json",
                          "tests/symbolic/hard-gos-04-imp-deceived.json"}) {
        const PlanningTask t = load(f);
        const AgentSymmetry sym = AgentSymmetry::detect(t);
        CHECK(!sym.empty());
        const auto R = explore(t, 30);
        sym::Engine eng(t);
        for (const auto& r : R) {
            sym::SymState s = eng.contract(eng.from_explicit(r.state));
            for (const auto& sw : sym.swaps) {
                AgentSymmetry one = sym;
                one.swaps = {sw};
                const bool exp = !stabiliser(one, r.state).trivial;
                CHECK_MSG(eng.fixes(s, sw) == exp, f);
                ++n;
                fixed += exp;
            }
        }
    }
    std::cout << "  " << n << " (state, swap) pairs, " << fixed << " fixing\n";
}

// ------------------------------------------------------------ hybrid search

// Replays a linear plan explicitly and checks the goal.
bool valid(const PlanningTask& t, const std::vector<std::string>& plan) {
    EpistemicState s = bisim_contract(t.init);
    for (const auto& name : plan) {
        const Action& a = t.actions[t.action_index.at(name)];
        if (!a.applicable(s)) return false;
        auto r = product_update(s, a, Seriality::Ignore);
        if (!r) return false;
        s = bisim_contract(std::move(*r));
    }
    return s.satisfies(*t.goal);
}

TEST(hybrid_search_finds_valid_plans_at_every_threshold) {
    for (const char* f : {"benchmarks/gossip1/problem_1.json", "benchmarks/coin1/problem_1.json",
                          "benchmarks/muddy1/muddy-children-problem.json",
                          "tests/symbolic/intermediate-gos-05-all.json",
                          "tests/symbolic/hard-gos-04-imp-deceived.json"}) {
        PlanningTask t = load(f);
        t.symmetry = std::make_shared<AgentSymmetry>(AgentSymmetry::detect(t));
        const KnowledgeRelaxationHeuristic h(t);
        std::cerr.setstate(std::ios::failbit);
        const auto ref = gbfs::search(t, h);
        std::cerr.clear();
        CHECK_MSG(ref.has_value(), f);
        for (double th : {0.0, 4.0, 4096.0}) {
            for (double hth : {0.0, 1e9}) {
                symsearch::Options o;
                o.threshold = th;
                o.hidden_threshold = std::max(th, hth);
                std::cerr.setstate(std::ios::failbit);
                const auto r = symsearch::search(t, h, o);
                std::cerr.clear();
                CHECK_MSG(r.has_value(), f);
                if (!r || !ref) continue;
                CHECK_MSG(valid(t, r->plan), f);
                // Same heuristic values and the same duplicate detection, so
                // the same search as the explicit planner.
                CHECK_MSG(r->plan.size() == ref->plan.size(), f + std::string(" threshold ") + std::to_string(th));
                CHECK_MSG(r->stats.nodes_expanded == ref->stats.nodes_expanded, f);
            }
        }
    }
}

// From a finitary S5 theory that is never materialised.
TEST(hybrid_search_from_theory) {
    setenv("ALETHEIA_THEORY_MAX_FREE", "0", 1);
    PlanningTask t = load("tests/symbolic/basic-gos-05-all.theory.json");
    unsetenv("ALETHEIA_THEORY_MAX_FREE");
    CHECK(t.theory.has_value());
    CHECK(t.init.num_worlds == 0);
    PlanningTask m = load("tests/symbolic/basic-gos-05-all.theory.json");   // materialised
    CHECK(m.init.num_worlds == 32);
    t.symmetry = std::make_shared<AgentSymmetry>(AgentSymmetry::detect(m));
    const KnowledgeRelaxationHeuristic h(t);
    for (double th : {0.0, 4096.0}) {
        symsearch::Options o;
        o.threshold = th;
        std::cerr.setstate(std::ios::failbit);
        const auto r = symsearch::search(t, h, o);
        std::cerr.clear();
        CHECK(r.has_value());
        if (r) CHECK(valid(m, r->plan));
    }
}

} // namespace

int main() { return run_all(); }
