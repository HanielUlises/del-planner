// The temporal planner's evaluation, one task at a time.
//
//   temporal_eval <task.json> [options]
//
//   --durations F    durations (seconds, or an eplansys action mapping);
//                    default: one second per action
//   --label L        name for the task in the output
//   --concurrency K  concurrency bound (default 3)
//   --timeout S      per search, seconds (default 120)
//   --runs N         noisy executions per policy (default 1000)
//   --seeds N        random-duration seeds, U[1,10] s (default 0)
//   --emit DIR       write each seed's durations to DIR/<label>-seed<i>.json,
//                    in the action-mapping shape, for the B1 tool
//
// Prints one JSON object per line: the nominal configuration, then one per
// seed. For each: the step-optimal untimed policy run in sequence (B3), the
// optimal temporal policy (ours), the satisficing one under max(h_adm, h_cp),
// the temporal policy with syntactic overlap (B2), and for ours (dispatched
// by events and by the clock) and B2 the share of noisy executions that fail.

#include "bisimulation.hpp"
#include "parser.hpp"
#include "search.hpp"
#include "strategy.hpp"
#include "temporal_execute.hpp"
#include "temporal_search.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <optional>
#include <iostream>
#include <random>
#include <string>

namespace {

using nlohmann::json;
using Clock = std::chrono::steady_clock;

// The longest branch of an untimed policy with its actions run one after
// another, and its number of steps.
std::pair<double, int> sequential(const std::shared_ptr<PlanNode>& node, const PlanningTask& task,
                                  const temporal::Durations& d)
{
    if (!node) return {0.0, 0};
    double worst = 0.0;
    int steps = 0;
    for (const auto& [event, child] : node->branches) {
        const auto [w, s] = sequential(child, task, d);
        worst = std::max(worst, w);
        steps = std::max(steps, s);
    }
    return {d[task.action_index.at(node->action)] + worst, steps + 1};
}

json search(const PlanningTask& task, const temporal::Durations& d, std::size_t k, double timeout,
            temporal::SearchOptions::Overlap overlap, temporal::SearchOptions::Estimate estimate,
            std::optional<temporal::TemporalPolicy>* out)
{
    temporal::SearchOptions o;
    o.max_concurrency = k;
    o.overlap  = overlap;
    o.estimate = estimate;
    o.deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                    std::chrono::duration<double>(timeout));
    temporal::SearchStats st;
    auto p = temporal::search_policy(task, d, o, &st);
    json j;
    j["found"]    = p.has_value();
    j["timeout"]  = !p && !st.exhausted;
    j["makespan"] = p ? json(p->makespan) : json(nullptr);
    j["leaves"]   = p ? p->leaves : 0;
    j["expanded"] = st.expanded;
    j["seconds"]  = st.elapsed_sec;
    j["start_checks"]      = st.start_checks;
    j["start_rejected"]    = st.start_rejected;
    j["commutation_checks"] = st.commutation_checks;
    j["start_check_seconds"] = st.start_check_sec;
    if (p && overlap == temporal::SearchOptions::Overlap::Commutation) {
        std::string why;
        j["valid"] = temporal::validate_policy(task, d, *p, &why);
        if (!why.empty()) j["invalid_because"] = why;
    }
    if (out) *out = std::move(p);
    return j;
}

json noisy(const PlanningTask& task, const temporal::TemporalPolicy& policy, std::size_t runs,
           temporal::Dispatch dispatch, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> noise(0.5, 1.5);
    std::vector<WorldIdx> worlds;
    for (WorldIdx w = 0; w < task.init.num_worlds; ++w)
        if (task.init.is_designated(w)) worlds.push_back(w);
    std::uniform_int_distribution<std::size_t> pick(0, worlds.size() - 1);

    std::size_t failed = 0;
    std::map<std::string, std::size_t> reasons;
    for (std::size_t r = 0; r < runs; ++r) {
        std::vector<double> factor;
        const auto actual = [&](std::size_t item, double planned) {
            while (factor.size() <= item) factor.push_back(noise(rng));
            return planned * factor[item];
        };
        const auto e = temporal::execute_policy(task, policy, actual, worlds[pick(rng)], dispatch);
        if (!e.ok) {
            ++failed;
            ++reasons[e.failure];
        }
    }
    json j;
    j["runs"] = runs;
    j["failed"] = failed;
    j["reasons"] = reasons;
    return j;
}

json configuration(const PlanningTask& task, const temporal::Durations& d,
                   const std::shared_ptr<PlanNode>& untimed, std::size_t k, double timeout,
                   std::size_t runs, unsigned seed)
{
    using O = temporal::SearchOptions;
    json j;
    const auto [b3, steps] = sequential(untimed, task, d);
    j["B3"] = {{"makespan", b3}, {"steps", steps}};

    std::optional<temporal::TemporalPolicy> ours, b2;
    j["ours"] = search(task, d, k, timeout, O::Overlap::Commutation, O::Estimate::Admissible, &ours);
    j["cp"]   = search(task, d, k, timeout, O::Overlap::Commutation, O::Estimate::CriticalPath, nullptr);
    j["B2"]   = search(task, d, k, timeout, O::Overlap::Syntactic, O::Estimate::Admissible, &b2);

    if (runs > 0) {
        if (ours) {
            j["noise"]["ours_events"] = noisy(task, *ours, runs, temporal::Dispatch::Events, seed);
            j["noise"]["ours_clock"]  = noisy(task, *ours, runs, temporal::Dispatch::Clock, seed);
        }
        if (b2) j["noise"]["B2_events"] = noisy(task, *b2, runs, temporal::Dispatch::Events, seed);
    }
    return j;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: temporal_eval <task.json> [--durations F] [--label L] "
                     "[--concurrency K] [--timeout S] [--runs N] [--seeds N] [--emit DIR]\n";
        return 2;
    }
    const std::string task_path = argv[1];
    std::string durations_path, label = task_path, emit;
    std::size_t k = 3, runs = 1000, seeds = 0;
    double timeout = 120.0;
    for (int i = 2; i + 1 < argc; i += 2) {
        const std::string a = argv[i], v = argv[i + 1];
        if (a == "--durations") durations_path = v;
        else if (a == "--label") label = v;
        else if (a == "--concurrency") k = std::stoul(v);
        else if (a == "--timeout") timeout = std::stod(v);
        else if (a == "--runs") runs = std::stoul(v);
        else if (a == "--seeds") seeds = std::stoul(v);
        else if (a == "--emit") emit = v;
    }

    const PlanningTask task = load_task(task_path);
    const auto h = make_heuristic("rpg", task);
    const auto untimed_start = Clock::now();
    bool exhausted = false;
    const auto untimed = aostar::search(task, *h, 0, Clock::now() + std::chrono::seconds(
                                            static_cast<long>(timeout)), &exhausted);
    if (!untimed) {
        std::cout << json{{"task", label}, {"error", "no untimed policy"}}.dump() << "\n";
        return 1;
    }
    const double untimed_seconds =
        std::chrono::duration<double>(Clock::now() - untimed_start).count();

    const temporal::Durations nominal = durations_path.empty()
        ? temporal::Durations::uniform(task) : temporal::load_durations(task, durations_path);

    json j = configuration(task, nominal, untimed->plan_tree, k, timeout, runs, 1);
    j["task"] = label;
    j["config"] = durations_path.empty() ? "unit" : "nominal";
    j["actions"] = task.actions.size();
    j["worlds"] = task.init.num_worlds;
    j["untimed_seconds"] = untimed_seconds;
    std::cout << j.dump() << "\n";

    std::mt19937 rng(20261122);
    std::uniform_real_distribution<double> uniform(1.0, 10.0);
    for (std::size_t s = 1; s <= seeds; ++s) {
        temporal::Durations d = temporal::Durations::uniform(task);
        for (auto& x : d.of) x = std::round(uniform(rng) * 10.0) / 10.0;
        if (!emit.empty()) {
            json m;
            for (std::size_t a = 0; a < task.actions.size(); ++a)
                m[task.actions[a].name] = {{"action", "(" + task.actions[a].name + ")"},
                                           {"duration", d.of[a]}};
            std::ofstream(emit + "/" + label + "-seed" + std::to_string(s) + ".json") << m.dump(1);
        }
        json js = configuration(task, d, untimed->plan_tree, k, timeout, runs / 10, 100 + s);
        js["task"] = label;
        js["config"] = "seed" + std::to_string(s);
        std::cout << js.dump() << "\n";
    }
    return 0;
}
