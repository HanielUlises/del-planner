// Tabulate temporal::commutes over the states of a task.
//
//   commutation_scan <task.json> [max_depth=4] [--witnesses]
//
// Walks the states reachable from the initial state by up to max_depth
// actions (every outcome of every applicable action, deduplicated by
// fingerprint), and in each one decides every pair of actions that are both
// applicable there. Reports how often the pairs commute, why they do not, and
// how the syntactic independence test compares: pairs it calls independent
// that do not commute (unsound for overlap) and pairs it rejects that do
// (overly cautious).

#include "bisimulation.hpp"
#include "parser.hpp"
#include "temporal.hpp"

#include <array>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <string>
#include <unordered_set>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: commutation_scan <task.json> [max_depth=4] [--witnesses]\n";
        return 2;
    }
    const std::string path = argv[1];
    std::size_t max_depth  = 4;
    bool witnesses         = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--witnesses") witnesses = true;
        else max_depth = std::strtoul(arg.c_str(), nullptr, 10);
    }

    const PlanningTask task = load_task(path);
    const WorldCapPolicy cap = make_world_cap_policy(task.partial_obs);

    struct Item { EpistemicState state; std::size_t depth; };
    std::deque<Item> frontier;
    std::unordered_set<Fingerprint, FingerprintHash> seen;

    EpistemicState init = bisim_contract(task.init);
    seen.insert(init.fingerprint());
    frontier.push_back({std::move(init), 0});

    std::array<std::size_t, 5> verdicts{};
    std::size_t states = 0, pairs = 0, updates = 0;
    std::size_t syn_indep = 0, unsound = 0, cautious = 0;

    while (!frontier.empty()) {
        Item item = std::move(frontier.front());
        frontier.pop_front();
        ++states;

        std::vector<ActionIdx> applicable;
        for (ActionIdx a = 0; a < task.actions.size(); ++a)
            if (task.actions[a].applicable(item.state)) applicable.push_back(a);

        for (std::size_t i = 0; i < applicable.size(); ++i) {
            for (std::size_t j = i + 1; j < applicable.size(); ++j) {
                const ActionIdx a = applicable[i], b = applicable[j];
                const auto c = temporal::commutes(task, item.state, a, b, cap);
                ++pairs;
                updates += c.updates;
                ++verdicts[static_cast<std::size_t>(c.verdict)];

                const bool syn = temporal::syntactically_independent(task.actions[a], task.actions[b]);
                if (syn) ++syn_indep;
                if (syn && !c.commute()) {
                    ++unsound;
                    if (witnesses) std::cout << "  unsound: " << c.explain(task) << "\n";
                }
                if (!syn && c.commute()) {
                    ++cautious;
                    if (witnesses) std::cout << "  cautious: " << c.explain(task) << "\n";
                }
                if (witnesses && c.verdict == temporal::Verdict::ResultsDiffer)
                    std::cout << "  results-differ: " << c.explain(task) << "\n";
            }
        }

        if (item.depth >= max_depth) continue;
        for (const auto& action : task.actions) {
            for (auto& [e, next] : temporal::outcomes(task, item.state, action, cap)) {
                if (seen.insert(next.fingerprint()).second)
                    frontier.push_back({std::move(next), item.depth + 1});
            }
        }
    }

    std::cout << "task           " << path << "\n"
              << "depth          " << max_depth << "\n"
              << "states         " << states << "\n"
              << "pairs          " << pairs << " (both applicable)\n"
              << "updates        " << updates << "\n";
    for (std::size_t v = 0; v < verdicts.size(); ++v)
        std::cout << "  " << temporal::verdict_name(static_cast<temporal::Verdict>(v)) << ": "
                  << verdicts[v] << "\n";
    std::cout << "syntactic      " << syn_indep << " pair(s) called independent\n"
              << "  unsound      " << unsound << " of them do not commute\n"
              << "  cautious     " << cautious << " pair(s) rejected that do commute\n";
    return 0;
}
