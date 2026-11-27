#pragma once
// Hybrid greedy best-first search: explicit states at or below a world
// threshold, symbolic (BDD) states above it.
//
// A node is symbolic when its contracted model has more than `threshold`
// worlds and is valuation-injective, or more than `hidden_threshold` worlds.
// Both are bisimulation invariants, so one epistemic state always gets the
// same representation and duplicate detection never compares across the two:
//
//   explicit   the planner's canonical fingerprint (bisim_contract)
//   symbolic   H = ∅: the structural digest of (θ, δ, Ω), canonical there
//              H ≠ ∅: the structural digest (same encoding), then states with
//              the same ∃H-invariant are compared exactly by bisimilarity on
//              their disjoint union; such states stay alive as GC roots
//
// Successors cross over when they land on the other side of the threshold:
// explicit → symbolic by encoding the contracted model, symbolic → explicit
// by enumerating it and contracting again (which restores the canonical
// numbering). The heuristic is the one the explicit search uses, evaluated on
// whichever representation the node has: kadd and kff give equal values on
// both (symheur.hpp); hd and hdsum use the symbolic h^Δ above the threshold,
// which is admissible but weaker than the explicit one; ug counts goal
// conjuncts. Symmetry pruning tests swaps by node equality on H = ∅ states.
// Linear plans only (GBFS), and only without seriality repair or the
// consistent-beliefs check, which the symbolic engine does not implement.
#include "heuristic.hpp"
#include "search.hpp"
#include "task.hpp"

#include <optional>
#include <string>

namespace symsearch {

struct Options {
    std::string heuristic = "kadd";   // kadd | kff | hd | hdsum | ug
    double      threshold = 4096;     // worlds: explicit at or below
    // Worlds at or below which a state that is not valuation-injective (H ≠ ∅
    // symbolically) stays explicit: hidden slots make BDDs large and the
    // contraction fixpoint slow, while injective states have a canonical,
    // small encoding (the observational states of the notes).
    double      hidden_threshold = 1 << 20;
    // When the contracted initial state has at least this many worlds, every
    // state of the run is symbolic (both thresholds 0): states then descend
    // from symbolic updates, whose hidden bits are structured (event codes,
    // old values), whereas encoding a large non-injective explicit state
    // numbers its worlds arbitrarily. 0 disables. The choice is made once per
    // run, so representations remain a function of the state.
    double      full_from = 1024;
    std::size_t max_nodes = 0;
    Deadline    deadline  = Deadline::max();
    bool        helpful   = true;
};

struct Stats {
    std::size_t explicit_generated{0}, symbolic_generated{0};
    std::size_t to_symbolic{0}, to_explicit{0};
    std::size_t symbolic_expanded{0};
    std::size_t hidden_generated{0};       // symbolic states with H ≠ ∅ after contraction
    std::size_t hidden_same_encoding{0};   // duplicates caught by the structural digest
    std::size_t hidden_bisimilar{0};       // duplicates caught only by the exact check
    std::size_t hidden_checks{0};          // exact checks run
    std::size_t hidden_closed{0};          // H ≠ ∅ states kept for those checks
    std::size_t symmetric_tests{0};
    std::size_t symmetric_inherited{0};   // fixing swaps carried from the parent, not tested
    bool        full_symbolic{false};      // the run chose every state symbolic
    std::size_t gc_runs{0};
    std::size_t peak_bdd_nodes{0};
    std::size_t peak_bdd_bytes{0};
    double      max_worlds{0};
    std::size_t contractions{0}, fixpoint_skipped{0}, fixpoint_runs{0}, fixpoint_rounds{0};
    double      t_update{0}, t_contract{0}, t_heuristic{0}, t_identity{0}, t_convert{0};
    double      t_goal{0}, t_symmetry{0}, t_preferred{0};   // goal and applicability tests, stabiliser, helpful actions
};

// Supported heuristic labels.
[[nodiscard]] bool supports(const std::string& heuristic);

[[nodiscard]] std::optional<SearchResult>
search(const PlanningTask& task, const Heuristic& h, const Options& opt, Stats* stats = nullptr);

} // namespace symsearch
