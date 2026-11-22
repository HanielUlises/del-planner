// Temporal planning for plans that do not branch: tests of search_linear.

#include "bisimulation.hpp"
#include "parser.hpp"
#include "temporal_search.hpp"

#include "check.hpp"

#include <cmath>
#include <string>

namespace {

std::string fixture(const std::string& file)
{
    return std::string(TEMPORAL_FIXTURE_DIR) + "/" + file;
}

bool overlap(const temporal::Step& a, const temporal::Step& b)
{
    return a.start < b.end() - 1e-9 && b.start < a.end() - 1e-9;
}

std::size_t overlapping_pairs(const temporal::TemporalPlan& plan)
{
    std::size_t n = 0;
    for (std::size_t i = 0; i < plan.steps.size(); ++i)
        for (std::size_t j = i + 1; j < plan.steps.size(); ++j)
            if (overlap(plan.steps[i], plan.steps[j])) ++n;
    return n;
}

std::string show(const PlanningTask& task, const temporal::TemporalPlan& plan)
{
    std::string out;
    for (const auto& s : plan.steps)
        out += "\n    " + std::to_string(s.start) + " " + task.actions[s.action].name + " [" +
               std::to_string(s.duration) + "]";
    return out + "\n    makespan " + std::to_string(plan.makespan);
}

// ── Durations ───────────────────────────────────────────────────────────────

TEST(durations_are_read_from_an_eplansys_mapping)
{
    const auto task = load_task(fixture("robot-fleet-depot.json"));
    const auto d    = temporal::load_durations(task, fixture("mapping-robot-fleet-depot.json"));
    CHECK(d[task.action_index.at("goto-north_r1")] == 30.0);
    CHECK(d[task.action_index.at("inspect-south_r2")] == 5.0);
    CHECK(d[task.action_index.at("report-north-clear_r3")] == 1.0);
}

TEST(a_task_without_durations_is_untimed)
{
    const auto task = load_task(fixture("robot-fleet.json"));
    const auto d    = temporal::load_durations(task, fixture("robot-fleet.json"));
    for (const double x : d.of) CHECK(x == 1.0);
}

// ── Search ──────────────────────────────────────────────────────────────────

TEST(the_corridor_looks_then_tells)
{
    // The map says the corridor is clear, r1 is at the junction and r2 is not.
    // Telling r2 (1 s) beats sending it to look (30 s + 5 s).
    const auto task = load_task(fixture("robot-fleet-stale-prior-lone.json"));
    const auto d    = temporal::load_durations(task, fixture("mapping-robot-fleet.json"));

    temporal::SearchStats stats;
    const auto plan = temporal::search_linear(task, d, {}, &stats);
    CHECK(plan.has_value());
    if (!plan) return;
    CHECK_MSG(std::abs(plan->makespan - 6.0) < 1e-9, show(task, *plan));
    std::string why;
    CHECK_MSG(temporal::validate_linear(task, d, *plan, &why), why);
}

TEST(two_robots_drive_and_look_at_the_same_time)
{
    // Both corridors known clear to the planner, unknown to the robots. One
    // robot per corridor drives (30 s) and looks (5 s), the two halves
    // overlapping, then each announces what it saw (1 s). Done one after
    // another that is 72 s; at the same time, 36 s.
    const auto task = load_task(fixture("robot-fleet-depot-known.json"));
    const auto d    = temporal::load_durations(task, fixture("mapping-robot-fleet-depot.json"));

    temporal::SearchStats stats;
    const auto plan = temporal::search_linear(task, d, {}, &stats);
    CHECK(plan.has_value());
    if (!plan) return;
    CHECK_MSG(std::abs(plan->makespan - 36.0) < 1e-9, show(task, *plan));
    CHECK_MSG(overlapping_pairs(*plan) >= 2, show(task, *plan));
    std::string why;
    CHECK_MSG(temporal::validate_linear(task, d, *plan, &why), why);
    std::cout << "    expanded " << stats.expanded << ", start checks " << stats.start_checks
              << ", rejected " << stats.start_rejected << ", "
              << stats.elapsed_sec << " s" << show(task, *plan) << "\n";
}

TEST(one_at_a_time_is_the_sum_of_the_durations)
{
    const auto task = load_task(fixture("robot-fleet-depot-known.json"));
    const auto d    = temporal::load_durations(task, fixture("mapping-robot-fleet-depot.json"));

    temporal::SearchOptions sequential;
    sequential.max_concurrency = 1;
    const auto plan = temporal::search_linear(task, d, sequential);
    CHECK(plan.has_value());
    if (!plan) return;
    CHECK_MSG(std::abs(plan->makespan - 72.0) < 1e-9, show(task, *plan));
    CHECK(overlapping_pairs(*plan) == 0);
}

TEST(unit_durations_count_epochs)
{
    // Untimed, the makespan is the number of decision epochs on the critical
    // path: drive, look, announce.
    const auto task = load_task(fixture("robot-fleet-depot-known.json"));
    const auto plan = temporal::search_linear(task, temporal::Durations::uniform(task));
    CHECK(plan.has_value());
    if (plan) CHECK_MSG(std::abs(plan->makespan - 3.0) < 1e-9, show(task, *plan));
}

// ── Validation ──────────────────────────────────────────────────────────────

TEST(the_validator_rejects_an_overlap_that_does_not_commute)
{
    // Both robots drive north; r1 looks. Then r1 announces the corridor clear
    // while r2 starts looking at it. Each is applicable on its own, but the
    // announcement makes r2 know, and looking requires that r2 does not yet
    // know: the two do not commute, so they must not overlap.
    const auto task = load_task(fixture("robot-fleet-depot-known.json"));
    const auto d    = temporal::load_durations(task, fixture("mapping-robot-fleet-depot.json"));
    const auto idx  = [&](const char* n) { return task.action_index.at(n); };

    temporal::TemporalPlan plan;
    plan.steps = {{idx("goto-north_r1"), 0, 30}, {idx("goto-north_r2"), 0, 30},
                  {idx("inspect-north_r1"), 30, 5},
                  {idx("report-north-clear_r1"), 35, 1}, {idx("inspect-north_r2"), 35, 5}};
    plan.makespan = 40;
    std::string why;
    CHECK(!temporal::validate_linear(task, d, plan, &why));
    CHECK_MSG(why.find("incoherent") != std::string::npos, why);

    // The same actions one after the other are fine up to the goal, which
    // this plan does not reach: south is never looked at.
    plan.steps.back().start = 36;
    plan.makespan = 41;
    CHECK(!temporal::validate_linear(task, d, plan, &why));
    CHECK_MSG(why.find("not applicable") != std::string::npos ||
              why.find("goal") != std::string::npos, why);
}

TEST(the_validator_accepts_what_the_search_returns_at_every_concurrency)
{
    const auto task = load_task(fixture("robot-fleet-depot-known.json"));
    const auto d    = temporal::load_durations(task, fixture("mapping-robot-fleet-depot.json"));
    double previous = std::numeric_limits<double>::infinity();
    for (std::size_t k = 1; k <= 3; ++k) {
        temporal::SearchOptions o;
        o.max_concurrency = k;
        const auto plan = temporal::search_linear(task, d, o);
        CHECK(plan.has_value());
        if (!plan) continue;
        std::string why;
        CHECK_MSG(temporal::validate_linear(task, d, *plan, &why), why);
        // More concurrency can only help.
        CHECK(plan->makespan <= previous + 1e-9);
        previous = plan->makespan;
    }
}

} // namespace

int main() { return run_all(); }
