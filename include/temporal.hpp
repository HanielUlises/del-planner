#pragma once
// When two epistemic actions may overlap in time.
//
// A durative epistemic action is checked for applicability when it starts and
// changes the model, by product update, when it ends. Two actions that overlap
// may end in either order, and whoever executes them cannot always choose: a
// robot's actual durations jitter. Overlap is therefore safe only when the
// order of their ends does not matter, which is a property of the two event
// models *in the state where they would overlap*:
//
//   for every outcome e of a and f of b,
//     b is applicable after a|e, a is applicable after b|f,
//     the outcomes reachable in one order are those reachable in the other,
//     and  M ⊗ a|e ⊗ b|f  ≅  M ⊗ b|f ⊗ a|e   (bisimilar).
//
// commutes() decides this exactly, for one state, by computing both orders,
// contracting each result to its bisimulation quotient and comparing canonical
// fingerprints. Product update preserves bisimulation, so the quotient is the
// right object to compare: bisimilar models satisfy the same formulas and
// admit the same continuations.
//
// syntactically_independent() is the cheap test a planner or executor reaches
// for instead: neither action writes an atom the other reads or writes. It is
// neither necessary nor sufficient here. Not sufficient, because sensing and
// announcements change what agents know without writing any atom, and an
// action whose precondition is about knowledge (say, that r1 does not yet know
// whether the corridor is blocked) is disabled by an announcement that writes
// nothing. Not necessary, because two writes of the same value commute.

#include "action.hpp"
#include "state.hpp"
#include "task.hpp"
#include "types.hpp"
#include "world_cap_policy.hpp"

#include <string>
#include <utility>
#include <vector>

namespace temporal {

// The outcomes of `a` in `s`: one contracted state per designated event that
// fires, tagged by that event. An ontic action has one. Empty when `a` is not
// applicable in `s`, or every branch was pruned (world cap, seriality).
[[nodiscard]] std::vector<std::pair<EventIdx, EpistemicState>>
outcomes(const PlanningTask& task, const EpistemicState& s, const Action& a,
         const WorldCapPolicy& cap);

enum class Verdict : std::uint8_t {
    Commute,            // every outcome pair agrees in both orders
    Inapplicable,       // a or b is not applicable in s, so they cannot overlap here
    DisabledByFirst,    // after some outcome of `first`, `second` is not applicable
    OutcomesDiffer,     // an outcome pair is reachable in one order and not the other
    ResultsDiffer,      // some outcome pair leads to non-bisimilar models
};

[[nodiscard]] const char* verdict_name(Verdict v) noexcept;

struct Commutation {
    Verdict verdict{Verdict::Commute};

    // A witness when the verdict is not Commute: which action came first, and
    // the outcomes involved. `first`/`second` are the two actions in the order
    // that exposed the difference.
    ActionIdx first{0};
    ActionIdx second{0};
    EventIdx  first_event{0};
    EventIdx  second_event{0};

    // Number of outcome pairs compared, and product updates performed: the
    // cost of the check, for reporting.
    std::size_t pairs{0};
    std::size_t updates{0};

    [[nodiscard]] bool commute() const noexcept { return verdict == Verdict::Commute; }

    // One line, in the task's own names.
    [[nodiscard]] std::string explain(const PlanningTask& task) const;
};

// Decide whether `a` and `b` commute in `s`. Symmetric in a and b up to the
// witness.
[[nodiscard]] Commutation
commutes(const PlanningTask& task, const EpistemicState& s, ActionIdx a, ActionIdx b,
         const WorldCapPolicy& cap);

[[nodiscard]] Commutation
commutes(const PlanningTask& task, const EpistemicState& s, ActionIdx a, ActionIdx b);

// Atoms an action reads (preconditions, postcondition guards, observability
// conditions; modal subformulas read the atoms under them) and writes
// (postcondition targets).
struct AtomFootprint {
    std::vector<AtomIdx> reads;
    std::vector<AtomIdx> writes;
};

[[nodiscard]] AtomFootprint footprint(const Action& a);

// Neither writes an atom the other reads or writes. The textbook test, kept as
// a baseline: see the header comment for why it is not the right one.
[[nodiscard]] bool syntactically_independent(const Action& a, const Action& b);

} // namespace temporal
