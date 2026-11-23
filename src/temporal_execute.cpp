#include "temporal_execute.hpp"

#include "product_update.hpp"
#include "world_cap_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace temporal {

namespace {

// One action started on one branch, with what it waits for and the
// observations that select its branch, as eplansys serialises a policy.
struct Item {
    ActionIdx                                     action{0};
    double                                        planned_start{0.0};
    double                                        duration{0.0};
    std::vector<std::size_t>                      after;
    std::vector<std::pair<std::size_t, EventIdx>> given;
};

bool same_time(double a, double b) { return std::llround(a * 1e6) == std::llround(b * 1e6); }

void flatten(const Epoch& epoch, std::vector<std::pair<std::size_t, Step>> running,
             const std::vector<std::size_t>& ended,
             const std::vector<std::pair<std::size_t, EventIdx>>& given, std::vector<Item>& out)
{
    for (const auto& step : epoch.starts) {
        out.push_back(Item{step.action, step.start, step.duration, ended, given});
        running.emplace_back(out.size() - 1, step);
    }
    if (epoch.branches.empty()) return;

    std::vector<std::pair<std::size_t, Step>> rest;
    std::vector<std::size_t> now_ended = ended;
    for (const auto& [item, step] : running) {
        if (same_time(step.end(), epoch.advance_to)) now_ended.push_back(item);
        else rest.emplace_back(item, step);
    }
    for (const auto& branch : epoch.branches) {
        auto path = given;
        for (const auto& [observer, event] : branch.observed) {
            for (const auto& [item, step] : running) {
                if (step.action == observer && same_time(step.end(), epoch.advance_to)) {
                    path.emplace_back(item, event);
                    break;
                }
            }
        }
        flatten(*branch.next, rest, now_ended, path, out);
    }
}

enum class Status : std::uint8_t { Pending, Running, Ran, Skipped };

} // namespace

Execution execute_policy(const PlanningTask& task, const TemporalPolicy& policy,
                         const std::function<double(std::size_t, double)>& actual,
                         WorldIdx world, Dispatch dispatch)
{
    Execution result;
    const auto fail = [&](std::string why) {
        result.failure = std::move(why);
        result.ok = false;
        return result;
    };

    std::vector<Item> items;
    if (policy.root) flatten(*policy.root, {}, {}, {}, items);

    std::vector<Status>   status(items.size(), Status::Pending);
    std::vector<EventIdx> observed(items.size(), 0);
    std::vector<double>   ends(items.size(), 0.0);

    const WorldCapPolicy cap = make_world_cap_policy(task.partial_obs);
    EpistemicState model = task.init;
    if (!model.is_designated(world)) return fail("the actual world is not designated initially");

    double t = 0.0;
    while (true) {
        // Start, or skip, everything whose conditions are decided.
        for (bool progress = true; progress;) {
            progress = false;
            for (std::size_t i = 0; i < items.size(); ++i) {
                if (status[i] != Status::Pending) continue;
                bool skip = false, ready = true;
                for (const std::size_t j : items[i].after) {
                    if (status[j] == Status::Skipped) skip = true;
                    else if (status[j] != Status::Ran && dispatch == Dispatch::Events) ready = false;
                }
                for (const auto& [j, event] : items[i].given) {
                    if (status[j] == Status::Skipped) skip = true;
                    else if (status[j] != Status::Ran) ready = false;
                    else if (observed[j] != event) skip = true;
                }
                if (skip) {
                    status[i] = Status::Skipped;
                    progress  = true;
                    continue;
                }
                if (dispatch == Dispatch::Clock && t + 1e-9 < items[i].planned_start) ready = false;
                if (!ready) continue;

                const Action& a = task.actions[items[i].action];
                if (!a.applicable(model)) return fail(a.name + " is not applicable when it starts");
                status[i] = Status::Running;
                ends[i]   = t + actual(i, items[i].duration);
                ++result.actions;
                progress = true;
            }
        }

        double next_end = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < items.size(); ++i)
            if (status[i] == Status::Running) next_end = std::min(next_end, ends[i]);

        if (!std::isfinite(next_end)) {
            const bool pending = std::any_of(status.begin(), status.end(),
                                             [](Status s) { return s == Status::Pending; });
            if (!pending) break;
            if (dispatch == Dispatch::Clock) {
                double next_start = std::numeric_limits<double>::infinity();
                for (std::size_t i = 0; i < items.size(); ++i)
                    if (status[i] == Status::Pending && items[i].planned_start > t + 1e-9)
                        next_start = std::min(next_start, items[i].planned_start);
                if (std::isfinite(next_start)) {
                    t = next_start;
                    continue;
                }
            }
            return fail("nothing is running and the rest waits for something that never ends");
        }

        // Under clock dispatch a planned start is an event too: it fires at
        // its time whether or not what it was planned after has ended.
        if (dispatch == Dispatch::Clock) {
            double next_start = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < items.size(); ++i)
                if (status[i] == Status::Pending && items[i].planned_start > t + 1e-9)
                    next_start = std::min(next_start, items[i].planned_start);
            if (next_start < next_end - 1e-9) {
                t = next_start;
                continue;
            }
        }

        // End everything ending now, in the order the policy writes them.
        t = next_end;
        for (std::size_t i = 0; i < items.size(); ++i) {
            if (status[i] != Status::Running || ends[i] > t) continue;
            const Action& a = task.actions[items[i].action];
            if (!a.applicable(model)) return fail(a.name + " is not applicable when it ends");

            std::vector<EventIdx> events(a.designated_events.begin(), a.designated_events.end());
            std::sort(events.begin(), events.end());
            EventIdx fired = 0;
            bool found = false;
            for (const EventIdx e : events) {
                if (e < a.events.size() && model.holds_at(*a.events[e].precondition, world)) {
                    fired = e;
                    found = true;
                    break;
                }
            }
            if (!found) return fail("no event of " + a.name + " can fire in the actual world");

            auto update = product_update_with_map(model, a, task.seriality(), cap);
            if (!update) return fail("the update of " + a.name + " was pruned");

            // The observed branch: the worlds where the fired event did.
            EpistemicState next = std::move(update->state);
            std::fill(next.designated.begin(), next.designated.end(), 0);
            next.invalidate();
            for (WorldIdx v = 0; v < model.num_worlds; ++v) {
                if (!model.is_designated(v)) continue;
                if (!model.holds_at(*a.events[fired].precondition, v)) continue;
                const WorldIdx u = update->at(v, fired);
                if (u != kNoWorld) next.set_designated(u);
            }
            const WorldIdx moved = update->at(world, fired);
            if (moved == kNoWorld) return fail("the actual world did not survive " + a.name);

            model       = std::move(next);
            world       = moved;
            status[i]   = Status::Ran;
            observed[i] = fired;
            result.makespan = t;
        }
    }

    if (!model.satisfies(*task.goal)) return fail("the goal does not hold at the end");
    result.ok = true;
    return result;
}

} // namespace temporal
