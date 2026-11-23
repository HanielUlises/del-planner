#pragma once
// Executing a temporal policy under actual durations.
//
// A temporal policy is planned with nominal durations. On a robot the actual
// ones differ, and overlapping actions may end in another order than planned.
// execute_policy replays one execution: it follows the actual world through
// every product update, so a sensing action observes what is actually the
// case, and it fails at the first point where the execution goes wrong:
//
//   - an action is not applicable when it starts or when it ends,
//   - no event of an ending action can fire at the actual world,
//   - an observation selects a branch the policy does not have, or
//   - the goal does not hold once nothing is left to run.
//
// Two dispatch rules. Events starts an action once every action on its branch
// planned to end before it has ended, the rule under which an admissible
// policy is sound for any positive durations. Clock starts an action at its
// planned time (once the observations selecting its branch are in), the rule
// that guarantee does not cover.

#include "state.hpp"
#include "task.hpp"
#include "temporal_search.hpp"
#include "types.hpp"

#include <functional>
#include <string>

namespace temporal {

enum class Dispatch : std::uint8_t { Events, Clock };

struct Execution {
    bool        ok{false};
    std::string failure;      // empty when ok
    double      makespan{0.0};
    std::size_t actions{0};   // actions run
};

// `actual(item, planned)` gives the actual duration of the item-th action
// started (items are numbered in the order the policy is written, parents
// first) from its planned duration. `world` is the actual world, a designated
// world of task.init.
[[nodiscard]] Execution execute_policy(const PlanningTask& task, const TemporalPolicy& policy,
                                       const std::function<double(std::size_t, double)>& actual,
                                       WorldIdx world, Dispatch dispatch);

} // namespace temporal
