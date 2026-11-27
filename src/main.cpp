#include "hdelta.hpp"
#include "bisimulation.hpp"
#include "parser.hpp"
#include "validator.hpp"
#include "product_update.hpp"
#include "search.hpp"
#include "heuristic.hpp"
#include "selection_policy.hpp"
#include "knowledge_relaxation.hpp"
#include "parallel.hpp"
#include "portfolio.hpp"
#include "signature.hpp"
#include "strategy.hpp"
#include "symmetry.hpp"
#include "symsearch.hpp"
#include "temporal_search.hpp"

#include <nlohmann/json.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <memory>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <optional>

static void write_plan_tree(std::ostream& out,
                            const std::shared_ptr<PlanNode>& node,
                            int indent = 0) {
    std::string pad(indent * 2, ' ');
    std::string pad2((indent + 1) * 2, ' ');
    std::string pad3((indent + 2) * 2, ' ');

    if (!node) {
        out << "null";
        return;
    }

    out << "{\n";
    out << pad2 << "\"action\": \"" << node->action << "\",\n";
    out << pad2 << "\"branches\": [\n";

    for (size_t i = 0; i < node->branches.size(); i++) {
        auto& [eid, child] = node->branches[i];

        out << pad3 << "{\n";
        out << pad3 << "  \"event\": " << eid << ",\n";
        out << pad3 << "  \"subtree\": ";
        write_plan_tree(out, child, indent + 3);
        out << "\n" << pad3 << "}";

        if (i + 1 < node->branches.size())
            out << ",";

        out << "\n";
    }

    out << pad2 << "]\n";
    out << pad << "}";
}

static void write_linear_plan(std::ostream& out,
                              const SearchResult& result) {
    out << "[";
    for (size_t i = 0; i < result.plan.size(); i++) {
        if (i > 0) out << ", ";
        out << "\"" << result.plan[i] << "\"";
    }
    out << "]\n";
}

static void usage(const char* prog) {
    std::cerr
        << "Usage:\n"
        << "  " << prog
        << " --task <task.json> --plan <plan.json> [options]\n"
        << "\n"
        << "Options:\n"
        << "  --task         Path to grounded JSON task\n"
        << "  --plan         Output plan file\n"
        << "  --heuristic    ug | ed | ks | wc | rpg | radd | kadd | kff  (default: auto)\n"
        << "  --strategy     gbfs | ehc | aostar | replan | portfolio  (default: auto)\n"
        << "  --policy       Selection-policy JSON; overrides the built-in\n"
        << "                 rules used to auto-select strategy and heuristic\n"
        << "  --print-policy Write the effective policy to stdout and exit\n"
        << "  --explain      Report which rule decided each auto-selection\n"
        << "  --limit        Max nodes / max depth (0 = unlimited)\n"
        << "  --timeout      Timeout in seconds\n"
        << "  --ehc          Force EHC (alias for --strategy ehc)\n"
        << "  --gbfs         Force GBFS (alias for --strategy gbfs)\n"
        << "  --conditional  Force AO* (alias for --strategy aostar)\n"
        << "  --no-symmetry  Disable agent-symmetry pruning\n"
        << "  --kd45-repair  Delete non-serial worlds after KD45 updates\n"
        << "  --consistent-beliefs\n"
        << "                 Refuse states in which an agent believes a contradiction\n"
        << "  --no-portfolio Auto-selected AO* keeps the whole budget\n"
        << "  --no-helpful   GBFS expands every action, not preferred ones first\n"
        << "  --dead-ends M  h^Δ dead-end detection: off, root, suspect (default), all\n"
        << "  --signature    Print the task's structural signature as JSON and exit\n"
        << "  --subjective A Branch only on what agent A observes: events A cannot tell\n"
        << "                 apart share one branch. 'auto': each action on what its\n"
        << "                 actor, the first agent it names, observes\n"
        << "  --threads      Worker threads (default: all cores; 1 = serial)\n"
        << "  --temporal     Plan with durations, minimising makespan, overlapping\n"
        << "                 actions only where their updates commute (linear plans)\n"
        << "  --durations F  Durations JSON for --temporal: name -> seconds, or an\n"
        << "                 eplansys action mapping (default: 1 s each)\n"
        << "  --max-concurrency N\n"
        << "                 Actions running at once under --temporal (default: 3)\n"
        << "  --symbolic     Symbolic search (GBFS; kadd, kff, hd, hdsum, ug): every state\n"
        << "                 as BDDs when the initial state has >= 1024 worlds, otherwise\n"
        << "                 BDDs only above --symbolic-threshold injective worlds\n"
        << "  --symbolic-threshold N\n"
        << "                 World count above which states are symbolic (default: 4096;\n"
        << "                 0 = every state symbolic)\n"
        << "  --symbolic-hidden-threshold N\n"
        << "                 The same for states that are not valuation-injective\n"
        << "                 (default: 1048576)\n"
        << "  --help         Show this message\n";
}

int main(int argc, char* argv[]) {
    // --timeout counts from process start: loading and grounding a large task
    // takes seconds, and a harness limits the whole run, not the search.
    const auto t_process = std::chrono::steady_clock::now();

    std::string task_path;
    std::string plan_path;
    std::string heuristic_name;  // empty = auto
    std::string strategy_name_arg;
    std::string policy_path;

    size_t limit        = 0;
    size_t timeout_secs = 0;

    bool print_policy = false;
    bool explain      = false;
    bool symmetry     = true;
    bool symbolic_mode = false;
    double symbolic_threshold = 4096;
    double symbolic_hidden = 1 << 20;
    bool symbolic_fixed = false;   // thresholds given: no per-task choice
    bool kd45_repair  = false;
    bool consistent   = false;
    bool portfolio_on = true;
    bool helpful_on   = true;
    bool signature    = false;
    std::string dead_ends = "suspect";
    bool temporal_mode    = false;
    std::string durations_path;
    size_t max_concurrency = 3;
    std::string subjective;   // empty = one branch per designated event

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if      (arg == "--task"      && i+1 < argc) task_path      = argv[++i];
        else if (arg == "--plan"      && i+1 < argc) plan_path      = argv[++i];
        else if (arg == "--heuristic" && i+1 < argc) heuristic_name = argv[++i];
        else if (arg == "--strategy"  && i+1 < argc) strategy_name_arg = argv[++i];
        else if (arg == "--policy"    && i+1 < argc) policy_path    = argv[++i];
        else if (arg == "--limit"     && i+1 < argc) limit          = std::stoul(argv[++i]);
        else if (arg == "--timeout"   && i+1 < argc) timeout_secs   = std::stoul(argv[++i]);
        else if (arg == "--print-policy") print_policy      = true;
        else if (arg == "--explain")      explain           = true;
        else if (arg == "--conditional")  strategy_name_arg = "aostar";
        else if (arg == "--ehc")          strategy_name_arg = "ehc";
        else if (arg == "--gbfs")         strategy_name_arg = "gbfs";
        else if (arg == "--no-symmetry")  symmetry          = false;
        else if (arg == "--kd45-repair")  kd45_repair       = true;
        else if (arg == "--consistent-beliefs") consistent  = true;
        else if (arg == "--no-portfolio") portfolio_on      = false;
        else if (arg == "--no-helpful")   helpful_on        = false;
        else if (arg == "--signature")    signature         = true;
        else if (arg == "--subjective" && i+1 < argc) subjective = argv[++i];
        else if (arg == "--dead-ends" && i+1 < argc) dead_ends  = argv[++i];
        else if (arg == "--threads"   && i+1 < argc) par::set_threads(std::stoul(argv[++i]));
        else if (arg == "--temporal")     temporal_mode     = true;
        else if (arg == "--durations" && i+1 < argc) durations_path = argv[++i];
        else if (arg == "--max-concurrency" && i+1 < argc) max_concurrency = std::stoul(argv[++i]);
        else if (arg == "--symbolic")     symbolic_mode     = true;
        else if (arg == "--symbolic-threshold" && i+1 < argc) { symbolic_mode = true; symbolic_fixed = true; symbolic_threshold = std::stod(argv[++i]); }
        else if (arg == "--symbolic-hidden-threshold" && i+1 < argc) { symbolic_mode = true; symbolic_fixed = true; symbolic_hidden = std::stod(argv[++i]); }
        else if (arg == "--help" || arg == "-h") { usage(argv[0]); return 0; }
        else {
            std::cerr << "Unknown argument: " << arg << "\n";
            usage(argv[0]);
            return 1;
        }
    }

    // Load the policy before anything else: a malformed one is a usage error,
    // and reporting it after a long parse would be needlessly late.
    SelectionPolicy policy;
    try {
        policy = policy_path.empty() ? SelectionPolicy::builtin()
                                     : SelectionPolicy::load(policy_path);
    } catch (const std::exception& e) {
        std::cerr << "Error in selection policy: " << e.what() << "\n";
        return 1;
    }

    if (print_policy) {
        std::cout << policy.to_json() << "\n";
        return 0;
    }

    if (task_path.empty() || plan_path.empty()) {
        std::cerr << "Error: --task and --plan are required.\n";
        usage(argv[0]);
        return 1;
    }

    PlanningTask task;
    // Under --symbolic an initial theory is materialised only when it is small
    // enough to start explicit; larger ones start from the theory on BDDs.
    if (symbolic_mode && !std::getenv("ALETHEIA_THEORY_MAX_FREE")) {
        unsigned free_bits = 0;
        while (free_bits < 24 && std::ldexp(1.0, int(free_bits) + 1) <= symbolic_threshold) ++free_bits;
        setenv("ALETHEIA_THEORY_MAX_FREE", std::to_string(free_bits).c_str(), 1);
    }
    try {
        task = load_task(task_path);
    } catch (const std::exception& e) {
        std::cerr << "Error loading task: " << e.what() << "\n";
        return 1;
    }

    // An initial theory with too many valuations is not materialised; only the
    // symbolic engine can start from it. Planning on the empty model would
    // satisfy every goal vacuously.
    if (task.theory && task.init.num_worlds == 0 && !symbolic_mode) {
        std::cerr << "Error: the initial state is a finitary S5 theory with too many valuations "
                     "to materialise (ALETHEIA_THEORY_MAX_FREE); use --symbolic\n";
        return 1;
    }

    if (subjective == "auto") {
        set_branching_agents(task.agent_names);
        symmetry = false;
        std::cerr << "[main] Subjective branching: each action branches on what its actor observes\n";
    } else if (!subjective.empty()) {
        const auto& names = task.agent_names;
        const auto it = std::find(names.begin(), names.end(), subjective);
        if (it == names.end()) {
            std::cerr << "Error: --subjective names no agent of the task: " << subjective << "\n";
            return 1;
        }
        set_branching_agent(static_cast<AgentIdx>(it - names.begin()));
        // An agent swap would carry the branching agent's perspective onto another's.
        symmetry = false;
        std::cerr << "[main] Subjective branching: policies branch on what " << subjective << " observes\n";
    }

    task.kd45_repair        = kd45_repair;
    task.consistent_beliefs = consistent;
    task.helpful_actions    = helpful_on;

    if (temporal_mode) {
        temporal::Durations durations = temporal::Durations::uniform(task);
        try {
            if (!durations_path.empty()) durations = temporal::load_durations(task, durations_path);
        } catch (const std::exception& e) {
            std::cerr << "Error loading durations: " << e.what() << "\n";
            return 1;
        }
        temporal::SearchOptions options;
        options.max_concurrency = max_concurrency;
        options.max_expansions  = limit;
        if (timeout_secs > 0) options.deadline = t_process + std::chrono::seconds(timeout_secs);

        temporal::SearchStats stats;
        const auto plan = temporal::search_linear(task, durations, options, &stats);
        std::cerr << "[temporal] expanded " << stats.expanded << ", generated " << stats.generated
                  << ", start checks " << stats.start_checks << " (" << stats.start_rejected
                  << " rejected), " << stats.elapsed_sec << " s\n";
        if (!plan) {
            // --temporal plans without branching. A task whose sensing has
            // several possible outcomes needs a policy, which
            // temporal::search_policy builds (eplansys calls it); say so
            // rather than call the task unsolvable.
            std::cerr << (stats.exhausted
                ? "[temporal] no linear plan exists; if the task's sensing has several "
                  "possible outcomes, it needs a branching temporal policy\n"
                : "[temporal] no plan found within the limits\n");
            return 1;
        }
        nlohmann::json out;
        out["temporal"] = true;
        out["makespan"] = plan->makespan;
        out["steps"]    = nlohmann::json::array();
        for (const auto& step : plan->steps)
            out["steps"].push_back({{"action", task.actions[step.action].name},
                                    {"start", step.start},
                                    {"duration", step.duration}});
        std::ofstream(plan_path) << out.dump(2) << "\n";
        std::cerr << "[temporal] " << plan->steps.size() << " actions, makespan "
                  << plan->makespan << "\n";
        return 0;
    }
    const TaskFeatures features = TaskFeatures::extract(task);

    if (symmetry) {
        // Symmetry only prunes, so detection gets a tenth of the limit and keeps
        // whatever swaps it verified in that time.
        const Deadline detect_by = timeout_secs > 0
            ? t_process + std::chrono::milliseconds(timeout_secs * 100) : Deadline::max();
        auto sym = std::make_shared<AgentSymmetry>(AgentSymmetry::detect(task, detect_by));
        std::cerr << "[symmetry] " << sym->swaps.size() << " agent swaps\n";
        if (!sym->empty()) task.symmetry = std::move(sym);
    }

    if (signature) {
        print_signature(task, std::cout);
        return 0;
    }

    if (explain) {
        std::cerr << "[main] Features:";
        for (auto& n : TaskFeatures::names())
            std::cerr << ' ' << n << '=' << *features.lookup(n);
        std::cerr << "\n";
    }

    // Heuristic selection: an explicit flag overrides the policy.
    std::string heuristic_label = heuristic_name;
    std::string heuristic_rule;

    if (heuristic_label.empty()) {
        Decision d    = select(policy.heuristic_rules, features);
        heuristic_label = d.outcome;
        heuristic_rule  = d.rule;
    }

    auto h = make_heuristic(heuristic_label, task);
    if (!h) {
        std::cerr << "Error: unknown heuristic '" << heuristic_label << "'; expected one of:";
        for (auto& l : heuristic_labels()) std::cerr << ' ' << l;
        std::cerr << "\n";
        return 1;
    }

    std::cerr << "[main] Heuristic: " << heuristic_display(heuristic_label);
    if (!heuristic_rule.empty()) {
        std::cerr << " (auto";
        if (explain) std::cerr << ", rule '" << heuristic_rule << "'";
        std::cerr << ")";
    }
    std::cerr << "\n";

    // Strategy selection: an explicit flag overrides the policy.
    std::string strategy_label = strategy_name_arg;
    std::string strategy_rule;

    if (strategy_label.empty()) {
        Decision d     = select(policy.strategy_rules, features);
        strategy_label = d.outcome;
        strategy_rule  = d.rule;
    }

    auto parsed = parse_strategy(strategy_label);
    if (!parsed) {
        std::cerr << "Error: unknown strategy '" << strategy_label << "'; expected one of:";
        for (auto& l : strategy_labels()) std::cerr << ' ' << l;
        std::cerr << "\n";
        return 1;
    }
    Strategy strategy = *parsed;

    if (!strategy_rule.empty()) {
        std::cerr << "[main] Strategy: " << strategy_name(strategy) << " (auto";
        if (explain) std::cerr << ", rule '" << strategy_rule << "'";
        std::cerr << ")\n";
    }

    using Clock = std::chrono::steady_clock;

    // One absolute deadline for every search and fallback below.
    const Deadline deadline = timeout_secs > 0
        ? t_process + std::chrono::seconds(timeout_secs) : Deadline::max();

    std::ofstream out(plan_path);
    if (!out.is_open()) {
        std::cerr << "Error: cannot open output file: " << plan_path << "\n";
        return 1;
    }

    // Dead ends (hdelta.hpp). h^Δ(s0) = ∞ proves that no plan and no policy
    // exists; the root test gets at most a tenth of the time limit, and an
    // unfinished test proves nothing.
    if      (dead_ends == "off")     task.dead_end_check = DeadEndCheck::Off;
    else if (dead_ends == "root")    task.dead_end_check = DeadEndCheck::Root;
    else if (dead_ends == "suspect") task.dead_end_check = DeadEndCheck::Suspect;
    else if (dead_ends == "all")     task.dead_end_check = DeadEndCheck::All;
    else { std::cerr << "Error: --dead-ends expects off, root, suspect or all\n"; return 1; }
    if (symbolic_mode) {
        // Hybrid explicit/symbolic GBFS (symsearch.hpp); does its own root test,
        // since the initial state may exist only as a finitary S5 theory.
        if (!symsearch::supports(heuristic_label)) {
            std::cerr << "Error: --symbolic supports the heuristics kadd, kff, hd, hdsum and ug, not '"
                      << heuristic_label << "'\n";
            return 1;
        }
        if (strategy != Strategy::GBFS)
            std::cerr << "[main] --symbolic plans linearly with GBFS; ignoring strategy "
                      << strategy_name(strategy) << "\n";
        std::cerr << "[main] Mode: hybrid GBFS (symbolic above " << symbolic_threshold << " worlds)\n";
        symsearch::Options so;
        so.heuristic = heuristic_label;
        so.threshold = symbolic_threshold;
        so.hidden_threshold = std::max(symbolic_threshold, symbolic_hidden);
        if (symbolic_fixed) so.full_from = 0;
        so.max_nodes = limit;
        so.deadline  = deadline;
        so.helpful   = helpful_on;
        symsearch::Stats ss;
        std::optional<SearchResult> result;
        try {
            result = symsearch::search(task, *h, so, &ss);
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
            return 1;
        }
        const auto& rs = result ? result->stats : PlannerStats{};
        std::cerr << "[symsearch] explicit nodes " << ss.explicit_generated
                  << ", symbolic nodes " << ss.symbolic_generated
                  << " (expanded " << ss.symbolic_expanded << ", H != 0: " << ss.hidden_generated
                  << "), conversions " << ss.to_symbolic << " up / " << ss.to_explicit << " down\n"
                  << "[symsearch] duplicates with H != 0: " << ss.hidden_same_encoding
                  << " same encoding, " << ss.hidden_bisimilar << " by bisimilarity ("
                  << ss.hidden_checks << " checks, " << ss.hidden_closed << " kept)\n"
                  << "[symsearch] contractions " << ss.contractions << " (fixpoint skipped "
                  << ss.fixpoint_skipped << ", run " << ss.fixpoint_runs << ", "
                  << ss.fixpoint_rounds << " rounds); gc " << ss.gc_runs
                  << "; peak BDD nodes " << ss.peak_bdd_nodes << " (" << ss.peak_bdd_bytes / (1 << 20)
                  << " MiB); max worlds " << ss.max_worlds << "\n"
                  << "[symsearch] time: update " << ss.t_update << " s, contract " << ss.t_contract
                  << " s, heuristic " << ss.t_heuristic << " s, identity " << ss.t_identity
                  << " s, convert " << ss.t_convert << " s, goal/applicable " << ss.t_goal
                  << " s, symmetry " << ss.t_symmetry << " s, helpful " << ss.t_preferred << " s\n";
        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found.\n";
            return 0;
        }
        std::cerr << "[symsearch] Solution found. Length=" << result->plan.size()
                  << "  Expanded=" << rs.nodes_expanded << "  Generated=" << rs.nodes_generated
                  << "  Time=" << rs.elapsed_sec << " s\n";
        write_linear_plan(out, *result);
        std::cerr << "[main] Plan written to " << plan_path << "\n";
        return 0;
    }

    if (task.dead_end_check != DeadEndCheck::Off) {
        const auto budget = timeout_secs > 0 ? std::chrono::milliseconds(timeout_secs * 100)
                                             : std::chrono::milliseconds(30'000);
        const auto root = hdelta::h_delta_until(task, bisim_contract(task.init),
                                                std::min(deadline, Clock::now() + budget));
        if (root && *root == hdelta::kInf) {
            out << "null\n";
            std::cerr << "[hdelta] No solution exists: the relaxation proves the goal unreachable.\n";
            return 0;
        }
        if (root) std::cerr << "[hdelta] h(s0) = " << *root << "\n";
    }

    if (strategy == Strategy::PORTFOLIO) {
        std::cerr << "[main] Mode: portfolio\n";
        const KnowledgeRelaxationHeuristic relaxation(task);
        const KnowledgeSpreadHeuristic     spread;
        PortfolioOutcome o = race(task, relaxation, spread, deadline);

        if (o.linear) {
            write_linear_plan(out, *o.linear);
            std::cerr << "[main] Plan written to " << plan_path << " (" << o.member << ")\n";
        } else if (o.contingent) {
            if (!o.contingent->plan_tree) out << "[]\n";
            else { write_plan_tree(out, o.contingent->plan_tree); out << "\n"; }
            std::cerr << "[main] Conditional plan written to " << plan_path << " (" << o.member << ")\n";
            if (expired(deadline)) {
                std::cerr << "[validator] skipped: deadline reached\n";
            } else {
                auto vr = validate(task, o.contingent->plan_tree);
                std::cerr << (vr.valid ? "[validator] OK\n" : "[validator] FAILED — " + vr.error + "\n");
            }
        } else {
            out << "null\n";
            std::cerr << (o.unsolvable ? "[main] No solution found (no policy exists).\n"
                                       : "[main] No solution found.\n");
        }
        return 0;
    }

    if (strategy == Strategy::AOSTAR || strategy == Strategy::REPLAN) {
        std::cerr << "[main] Mode: " << strategy_name(strategy) << "\n";

        // Auto-selected AO* on a sensing task runs as a portfolio: a short AO*
        // pass keeps shallowest plans on easy tasks, then replan takes the
        // remaining time.
        const bool portfolio = portfolio_on && strategy == Strategy::AOSTAR &&
                               !strategy_rule.empty() && has_sensing_actions(task);
        const auto ao_budget = std::chrono::seconds(
            std::min<std::size_t>(5, timeout_secs > 0 ? std::max<std::size_t>(1, timeout_secs / 10) : 5));
        bool exhausted = false;

        auto result = strategy == Strategy::AOSTAR
            ? aostar::search(task, *h, limit,
                             portfolio ? std::min(deadline, Clock::now() + ao_budget) : deadline,
                             &exhausted)
            : replan::search(task, *h, deadline);

        if (!result && portfolio && !exhausted) {
            std::cerr << "[main] AO* budget spent — switching to replan\n";
            result = replan::search(task, *h, deadline);
        }

        if (!result) {
            // AO* exhausted its budget. For partial-plan-linear domains
            // (partial_obs=true, sensing=false) a conformant linear solution
            // may exist that AO* couldn't find within the time/depth budget.
            // GBFS with the remaining wall-clock budget has a different search
            // order and may succeed.
            // GBFS gets what is left of the same deadline. A deadline already
            // reached leaves nothing to spend.
            if (!has_sensing_actions(task) && !expired(deadline)) {
                std::cerr << "[main] AO* failed — falling back to GBFS\n";

                auto gbfs_result = gbfs::search(task, *h, limit, deadline);
                if (gbfs_result) {
                    write_linear_plan(out, *gbfs_result);
                    std::cerr << "[main] Plan written to " << plan_path << "\n";
                    return 0;
                }
            }

            out << "null\n";
            std::cerr << "[main] No solution found.\n";
            return 0;
        }

        // An empty conditional plan means the goal already holds. Writing it as
        // "null" would make it indistinguishable from "no plan exists", which is
        // what the failure path emits; the empty array matches the convention
        // linear plans already use.
        if (!result->plan_tree) {
            out << "[]\n";
            std::cerr << "[main] Goal already satisfied — empty plan written to "
                      << plan_path << "\n";
        } else {
            write_plan_tree(out, result->plan_tree);
            out << "\n";
            std::cerr << "[main] Conditional plan written to " << plan_path << "\n";
        }

        // The plan is already written; checking it must not outlast the run.
        if (expired(deadline)) {
            std::cerr << "[validator] skipped: deadline reached\n";
        } else {
            auto vr = validate(task, result->plan_tree);
            if (vr.valid)
                std::cerr << "[validator] OK — " << vr.leaves_reached
                          << " leaves, " << vr.branches_checked << " branches checked\n";
            else
                std::cerr << "[validator] FAILED — " << vr.error << "\n";
        }

    } else if (strategy == Strategy::EHC) {
        std::cerr << "[main] Mode: EHC\n";

        auto result = ehc::search(task, *h, limit, deadline);
        if (!result) {
            std::cerr << "[main] EHC failed — falling back to GBFS\n";
            result = gbfs::search(task, *h, limit, deadline);
        }

        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found.\n";
            return 0;
        }

        write_linear_plan(out, *result);
        std::cerr << "[main] Plan written to " << plan_path << "\n";

    } else {
        std::cerr << "[main] Mode: GBFS\n";

        auto result = gbfs::search(task, *h, limit, deadline);
        if (!result) {
            out << "null\n";
            std::cerr << "[main] No solution found.\n";
            return 0;
        }

        write_linear_plan(out, *result);
        std::cerr << "[main] Plan written to " << plan_path << "\n";
    }

    return 0;
}