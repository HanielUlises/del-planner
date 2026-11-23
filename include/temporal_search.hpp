#pragma once
// Temporal planning over epistemic states, for plans that do not branch.
//
// Actions take time. An action is checked when it starts and updates the
// model when it ends; several may run at once, but only when their updates
// commute in every state the running set can pass through (see temporal.hpp).
// That condition, coherence, is what makes the belief at any moment
// independent of the order in which overlapping actions happen to end, so a
// plan stays valid whatever the actual durations, as long as the executor
// starts each action when the actions it waits for have ended.
//
// The search is over decision epochs. A node is (M, R, t): the model, the
// running actions with their end times, and the current time. Two moves:
//
//   start b   b applicable and the running set stays coherent with b in it;
//             no time passes
//   advance   time moves to the earliest end, and every action ending then
//             updates the model
//
// A goal node has nothing running and the goal holding. The cost is the time
// at which that happens, the makespan, which A* minimises with an admissible
// estimate.
//
// This is the linear case: every action must have exactly one outcome in every
// state it can end in. Branching on sensing outcomes is the AND-OR search that
// builds on this one.

#include "search.hpp"
#include "state.hpp"
#include "task.hpp"
#include "temporal.hpp"
#include "types.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace temporal {

// Duration of each action, indexed like task.actions.
struct Durations {
    std::vector<double> of;

    [[nodiscard]] double operator[](ActionIdx a) const { return of[a]; }

    // Every action takes `d`. An untimed task planned with unit durations
    // gets makespan = number of epochs.
    [[nodiscard]] static Durations uniform(const PlanningTask& task, double d = 1.0);
};

// Read durations from a JSON file, keyed by grounded action name. A value is
// a number of seconds, or an object with a "duration" field, which is the
// shape of an eplansys action mapping, so a mapping file can be passed as it
// is. A task file with a top-level "durations" object is read the same way.
// Actions the file does not name take `fallback`. Throws on a non-positive
// duration or a name the task does not have.
[[nodiscard]] Durations load_durations(const PlanningTask& task, const std::string& path,
                                       double fallback = 1.0);

struct Step {
    ActionIdx action{0};
    double    start{0.0};
    double    duration{0.0};

    [[nodiscard]] double end() const noexcept { return start + duration; }
};

struct TemporalPlan {
    std::vector<Step> steps;    // by start time, then action index
    double            makespan{0.0};
};

struct SearchOptions {
    // How many actions may run at once. The coherence check enumerates the
    // subsets of the running set, so its cost is exponential in this.
    std::size_t max_concurrency{3};
    // Weight on the estimate: 1 is A*, optimal; above 1 trades makespan for
    // speed, within that factor of optimal.
    double weight{1.0};
    std::chrono::steady_clock::time_point deadline{std::chrono::steady_clock::time_point::max()};
    std::size_t max_expansions{0};    // 0 = no limit

    // Which test decides whether an action may start while others run.
    // Commutation is the exact test, and the only one under which the
    // guarantees hold. Syntactic overlaps any two actions that neither write
    // an atom the other reads or writes: the textbook test, kept as a
    // baseline for evaluation (search_policy only).
    enum class Overlap : std::uint8_t { Commutation, Syntactic };
    Overlap overlap{Overlap::Commutation};

    // The estimate the policy search deepens with. Admissible is h_adm, and
    // the result is optimal. CriticalPath is max(h_adm, h_cp), where h_cp
    // weighs each layer of the relaxed closure by the shortest duration among
    // the actions that contribute to it (a running action by its remaining
    // time). It is not admissible: the policy returned is valid but need not
    // be optimal (search_policy only).
    enum class Estimate : std::uint8_t { Admissible, CriticalPath };
    Estimate estimate{Estimate::Admissible};

    // How the policy search's time bound grows between iterations: the next
    // bound is max(the root's lower bound, bound * bound_growth). Optimality
    // does not depend on it, since each iteration returns the exact least
    // cost within its bound; a larger factor trades fewer iterations for a
    // last iteration that searches past the optimum. 1 steps to the next
    // lower bound, one iteration per distinct reachable makespan.
    double bound_growth{1.0};
};

struct SearchStats {
    std::size_t expanded{0};
    std::size_t generated{0};
    std::size_t duplicates{0};
    std::size_t start_checks{0};      // coherence checks for a candidate start
    std::size_t start_rejected{0};    // candidates that would break coherence
    std::size_t commutation_checks{0};
    double      start_check_sec{0.0};   // time spent deciding starts, of elapsed_sec
    std::size_t iterations{0};          // deepening iterations (policy search)
    std::size_t dominated{0};           // nodes pruned by an earlier-ending sibling
    double      elapsed_sec{0.0};
    bool        exhausted{false};     // the whole space was searched
};

// The plan with the least makespan, or nullopt when there is none (or the
// deadline or expansion limit was reached first; `stats.exhausted` tells
// them apart).
[[nodiscard]] std::optional<TemporalPlan>
search_linear(const PlanningTask& task, const Durations& durations,
              const SearchOptions& options = {}, SearchStats* stats = nullptr);

// Replay a plan, with every action ending at its planned time, and check that
// every start is applicable, every end has exactly one outcome, the running
// set stays coherent, and the goal holds at the end. On failure, `why` says
// which step.
[[nodiscard]] bool validate_linear(const PlanningTask& task, const Durations& durations,
                                   const TemporalPlan& plan, std::string* why = nullptr);

// Is the running set R coherent in s: for every subset of R ended (in index
// order), every remaining action is applicable with exactly one outcome and
// every remaining pair commutes? This is the invariant search_linear keeps,
// exposed for tests.
[[nodiscard]] bool coherent(const PlanningTask& task, const EpistemicState& s,
                            const std::vector<ActionIdx>& running);

// ── Branching: temporal policies ────────────────────────────────────────────
//
// With sensing, what an action observed is known when it ends, and the policy
// branches there. A policy is a tree of decision epochs: at each, some
// actions start; then time advances to the next end, and the continuation is
// chosen by what the actions ending then observed.

struct Epoch;

// One continuation, selected by the outcomes observed at an advance: one
// (action, event) per action that ended then with more than one possible
// outcome. Actions with a single outcome are not listed.
struct Branch {
    std::vector<std::pair<ActionIdx, EventIdx>> observed;
    std::shared_ptr<const Epoch>                next;
};

struct Epoch {
    double            t{0.0};
    std::vector<Step> starts;     // started at t, by action index
    // After the starts, time advances to `advance_to` and one of `branches`
    // follows. No branches: nothing is running and the goal holds.
    double              advance_to{0.0};
    std::vector<Branch> branches;
};

struct TemporalPolicy {
    std::shared_ptr<const Epoch> root;
    double makespan{0.0};          // worst case over branches
    std::size_t leaves{0};
};

// The policy with the least worst-case makespan, among policies that start
// actions at decision epochs and overlap only coherent running sets.
// Iterative deepening on a time bound over the AND-OR graph, with results
// memoised by (model fingerprint, running actions with remaining times),
// which determines the time still needed whatever the absolute time is.
[[nodiscard]] std::optional<TemporalPolicy>
search_policy(const PlanningTask& task, const Durations& durations,
              const SearchOptions& options = {}, SearchStats* stats = nullptr);

// An untimed policy (a tree of actions branching on sensing outcomes, as
// aostar::search returns), scheduled as a temporal policy: each action of a
// branch starts as early as StartCheck allows, after the sensing action whose
// outcome selects it has ended, with at most `max_concurrency` running. Every
// start satisfies StartCheck, so the result is admissible, and no action
// starts later than it would run in sequence, so its makespan is at most the
// untimed policy's run in sequence. Not optimal: it keeps the untimed policy's
// actions. It is what the planner returns when the optimal search does not
// finish in time. nullopt only if the untimed policy does not reach the goal.
[[nodiscard]] std::optional<TemporalPolicy>
schedule_untimed(const PlanningTask& task, const Durations& durations,
                 const std::shared_ptr<PlanNode>& tree, std::size_t max_concurrency = 3);

// Replay every branch of a policy: every start applicable and coherent with
// what runs, every outcome combination an advance can produce covered by a
// branch, the goal holding with nothing running at every leaf, and the
// worst-case makespan as stated.
[[nodiscard]] bool validate_policy(const PlanningTask& task, const Durations& durations,
                                   const TemporalPolicy& policy, std::string* why = nullptr);

} // namespace temporal
