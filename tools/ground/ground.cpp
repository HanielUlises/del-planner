// ground — EPDDL to Aletheia JSON, with compact accessibility relations.
//
// Grounds a task with plank's libraries and writes the JSON of `plank export`,
// except that each agent's relation in the initial state is a table of distinct
// successor sets:
//
//   "relations": { "A": { "sets": [["w0", "w2"], ["w1", "w3"]],
//                         "of":   [0, 1, 0, 1] }, ... }
//
// "of" lists, for each world in the order of "worlds", the index of its
// successor set. On S5 and KD45 models an agent's sets are disjoint, so the
// relation costs O(|W|) instead of one name per edge: 1.3 MB instead of about
// 9 GB for IεPC gos-13-all. plank's exporter also builds the whole document in
// memory before writing it, which does not fit for that task; this tool writes
// the initial state as it goes.
//
//   ground -d domain.epddl -p problem.epddl [-l library.epddl ...] -o task.json
//   ground export -d ... -p ... [-l ...] -o dir     (as plank: dir/<problem>.json)
//
// With --theory, a problem whose initial state is a finitary S5 theory is
// written without building the state: plank's grounder enumerates every
// valuation and compares every pair of worlds, which does not scale past a
// few thousand worlds. The JSON then carries, instead of "initial-state",
//
//   "initial-theory": { "type-1": [φ, …],          designated: worlds satisfying all
//                       "type-2": [φ, …],          worlds: valuations satisfying all
//                       "type-3": { "A": [φ, …] }, R_A(w,v) iff w, v agree on every φ
//                       "fixed":  { "p": true } }  fact atoms, constant in every world
//
// which is exactly the data plank's finitary_s5_theory_grounder builds the
// state from (type-2 includes the C(K_i φ) formulas, as in plank).

#include "epddl/grounder/grounder_helper.h"
#include "epddl/grounder/actions/actions_grounder.h"
#include "epddl/grounder/formulas/formulas_and_lists_grounder.h"
#include "epddl/grounder/initial_state/finitary_s5_theory_grounder.h"
#include "epddl/type-checker/type_checker.h"
#include "epddl/json-printer/actions_printer.h"
#include "epddl/json-printer/facts_printer.h"
#include "epddl/json-printer/formulas_printer.h"
#include "epddl/json-printer/language_printer.h"
#include "epddl/error-manager/epddl_exception.h"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace plank;
using nlohmann::json;
using nlohmann::ordered_json;

namespace {

std::uint64_t mix64(std::uint64_t x) {
    x ^= x >> 30; x *= 0xBF58476D1CE4E5B9ULL;
    x ^= x >> 27; x *= 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

std::string quoted(const std::string& s) { return json(s).dump(); }

ordered_json task_info(const del::planning_task& task, const epddl::grounder::grounder_info& info) {
    ordered_json j;
    json libraries = json::array(), requirements = json::array();
    for (const auto& lib : info.context.components_names.get_libraries_names()) libraries.emplace_back(lib);
    for (const auto& req : info.context.requirements.get_total_requirements()) requirements.emplace_back(req);

    j["problem"]               = info.context.components_names.get_problem_name();
    j["domain"]                = info.context.components_names.get_domain_name();
    j["libraries"]             = libraries;
    j["requirements"]          = requirements;
    j["agents-number"]         = info.language->get_agents_number();
    j["atoms-number"]          = info.language->get_atoms_number();
    j["facts-number"]          = info.facts.size();
    j["actions-number"]        = task.actions.size();
    j["initial-worlds-number"] = task.initial_state->get_worlds_number();
    j["goal-modal-depth"]      = del::formulas_utils::get_modal_depth(task.goal);
    j["goal-size"]             = del::formulas_utils::get_size(task.goal);
    j["relations-format"]      = "successor-sets";
    return j;
}

void write_initial_state(std::ostream& out, const del::state_ptr& s) {
    const auto language = s->get_language();
    const del::world_id nw = s->get_worlds_number();

    out << "{\n    \"worlds\": [";
    for (del::world_id w = 0; w < nw; ++w)
        out << (w ? ", " : "") << quoted(s->get_world_name(w));
    out << "],\n    \"relations\": {";

    std::vector<del::world_id> succ;
    for (del::agent i = 0; i < language->get_agents_number(); ++i) {
        // Distinct successor sets, interned by content in first-occurrence order.
        std::vector<std::vector<del::world_id>> sets;
        std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> index;
        std::vector<std::uint32_t> of(nw);

        for (del::world_id w = 0; w < nw; ++w) {
            succ.assign(s->get_agent_possible_worlds(i, w).begin(), s->get_agent_possible_worlds(i, w).end());
            std::sort(succ.begin(), succ.end());
            std::uint64_t h = mix64(succ.size());
            for (del::world_id v : succ) h = mix64(h ^ v);

            auto& candidates = index[h];
            std::uint32_t id = UINT32_MAX;
            for (std::uint32_t k : candidates)
                if (sets[k] == succ) { id = k; break; }
            if (id == UINT32_MAX) {
                id = static_cast<std::uint32_t>(sets.size());
                sets.push_back(succ);
                candidates.push_back(id);
            }
            of[w] = id;
        }

        out << (i ? "," : "") << "\n        " << quoted(language->get_agent_name(i)) << ": {\n            \"sets\": [";
        for (std::size_t k = 0; k < sets.size(); ++k) {
            out << (k ? ", " : "") << "[";
            for (std::size_t m = 0; m < sets[k].size(); ++m)
                out << (m ? ", " : "") << quoted(s->get_world_name(sets[k][m]));
            out << "]";
        }
        out << "],\n            \"of\": [";
        for (del::world_id w = 0; w < nw; ++w) out << (w ? ", " : "") << of[w];
        out << "]\n        }";
    }
    out << "\n    },\n    \"labels\": {";

    for (del::world_id w = 0; w < nw; ++w) {
        out << (w ? "," : "") << "\n        " << quoted(s->get_world_name(w)) << ": [";
        bool first = true;
        for (del::atom p = 0; p < language->get_atoms_number(); ++p)
            if (s->get_label(w)[p]) {
                out << (first ? "" : ", ") << quoted(language->get_atom_name(p));
                first = false;
            }
        out << "]";
    }
    out << "\n    },\n    \"designated\": [";
    std::vector<del::world_id> designated(s->get_designated_worlds().begin(), s->get_designated_worlds().end());
    std::sort(designated.begin(), designated.end());
    for (std::size_t k = 0; k < designated.size(); ++k)
        out << (k ? ", " : "") << quoted(s->get_world_name(designated[k]));
    out << "]\n}";
}

int usage() {
    std::cerr << "usage: ground [export] [--theory] -d domain.epddl -p problem.epddl [-l library.epddl ...] -o task.json|dir\n";
    return 2;
}

// --theory: ground everything but the initial state, and write the theory.
int write_theory_task(const epddl::parser::specification_paths& spec_paths, const std::string& output) {
    using namespace plank::epddl;
    auto [spec, err_managers] = parser::file_parser::parse_planning_specification(spec_paths, true);
    type_checker::context context = type_checker::do_semantic_check(spec, err_managers);
    grounder::grounder_info info = grounder::grounder_helper::build_info(spec, context, std::move(err_managers));

    const ast::finitary_S5_theory* init = nullptr;
    for (const auto& item : std::get<0>(spec)->get_items())
        if (std::holds_alternative<ast::initial_state_ptr>(item)) {
            const auto& st = std::get<ast::initial_state_ptr>(item)->get_state();
            if (std::holds_alternative<ast::finitary_S5_theory>(st)) init = &std::get<ast::finitary_S5_theory>(st);
        }
    if (!init) {
        std::cerr << "--theory: the initial state is not a finitary S5 theory\n";
        return 3;
    }
    const auto theory = grounder::finitary_s5_theory_grounder::build_finitary_s5_theory(*init, info);
    auto [actions_names, actions_map, actions] = grounder::actions_grounder::build_actions(spec, info);
    const auto goal = grounder::formulas_and_lists_grounder::build_goal(spec, info);
    const auto& language = info.language;

    ordered_json ti;
    json libraries = json::array(), requirements = json::array();
    for (const auto& lib : info.context.components_names.get_libraries_names()) libraries.emplace_back(lib);
    for (const auto& req : info.context.requirements.get_total_requirements()) requirements.emplace_back(req);
    ti["problem"]              = info.context.components_names.get_problem_name();
    ti["domain"]               = info.context.components_names.get_domain_name();
    ti["libraries"]            = libraries;
    ti["requirements"]         = requirements;
    ti["agents-number"]        = language->get_agents_number();
    ti["atoms-number"]         = language->get_atoms_number();
    ti["facts-number"]         = info.facts.size();
    ti["actions-number"]       = actions.size();
    ti["initial-state-format"] = "finitary-s5-theory";
    ti["goal-modal-depth"]     = del::formulas_utils::get_modal_depth(goal);
    ti["goal-size"]            = del::formulas_utils::get_size(goal);

    auto formulas = [&](const del::formula_deque& fs) {
        json a = json::array();
        for (const auto& f : fs) a.emplace_back(printer::formulas_printer::build_formula_json(language, f));
        return a;
    };
    ordered_json th, t3, fixed;
    th["type-1"] = formulas(theory->get_type_1_formulas());
    th["type-2"] = formulas(theory->get_type_2_formulas());
    for (del::agent i = 0; i < language->get_agents_number(); ++i)
        t3[language->get_agent_name(i)] = formulas(theory->get_type_3_formulas(i));
    th["type-3"] = t3;
    fixed = json::object();
    for (del::atom p = 0; p < language->get_atoms_number(); ++p)
        if (language->is_fact(p)) fixed[language->get_atom_name(p)] = bool(info.facts.get_bitset()[p]);
    th["fixed"] = fixed;

    std::ofstream out(output);
    if (!out) {
        std::cerr << "cannot write " << output << "\n";
        return 1;
    }
    out << "{\n\"planning-task-info\": " << ti.dump(2)
        << ",\n\"language\": " << printer::language_printer::build_language_json(language).dump(2)
        << ",\n\"facts\": " << printer::facts_printer::build_facts_json(info.language, info.facts).dump(2)
        << ",\n\"initial-theory\": " << th.dump(2)
        << ",\n\"actions\": " << printer::actions_printer::build_actions_json(actions_names, actions_map).dump(2)
        << ",\n\"goal\": " << printer::formulas_printer::build_formula_json(language, goal).dump(2)
        << "\n}\n";
    if (!out) {
        std::cerr << "write failed: " << output << "\n";
        return 1;
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::string domain, problem, output;
    std::vector<std::string> libraries;
    bool theory = false;
    int first = 1;
    const bool plank_style = argc > 1 && std::string(argv[1]) == "export";
    if (plank_style) first = 2;
    for (int k = first; k < argc; ++k) {
        const std::string a = argv[k];
        if ((a == "-d" || a == "--domain") && k + 1 < argc) domain = argv[++k];
        else if ((a == "-p" || a == "--problem") && k + 1 < argc) problem = argv[++k];
        else if ((a == "-o" || a == "--output") && k + 1 < argc) output = argv[++k];
        else if (a == "--theory") theory = true;
        else if (a == "-l" || a == "--libraries") {
            while (k + 1 < argc && argv[k + 1][0] != '-') libraries.push_back(argv[++k]);
        } else return usage();
    }
    if (domain.empty() || problem.empty() || output.empty()) return usage();
    // plank export takes a directory and names the file after the problem.
    if (plank_style || std::filesystem::is_directory(output)) {
        std::filesystem::create_directories(output);
        output = (std::filesystem::path(output) / (std::filesystem::path(problem).stem().string() + ".json")).string();
    }

    const auto [spec_paths, failed] =
        epddl::grounder::grounder_helper::get_specification_paths(domain, problem, libraries, "");
    if (failed) return 1;

    try {
        if (theory) return write_theory_task(spec_paths, output);
        const auto [task, info] = epddl::grounder::grounder_helper::ground(spec_paths, true);
        const auto language = task.initial_state->get_language();

        std::ofstream out(output);
        if (!out) {
            std::cerr << "cannot write " << output << "\n";
            return 1;
        }
        out << "{\n\"planning-task-info\": " << task_info(task, info).dump(2)
            << ",\n\"language\": " << printer::language_printer::build_language_json(language).dump(2)
            << ",\n\"facts\": " << printer::facts_printer::build_facts_json(info.language, info.facts).dump(2)
            << ",\n\"initial-state\": ";
        write_initial_state(out, task.initial_state);
        out << ",\n\"actions\": "
            << printer::actions_printer::build_actions_json(task.actions_names, task.actions_map).dump(2)
            << ",\n\"goal\": " << printer::formulas_printer::build_formula_json(language, task.goal).dump(2)
            << "\n}\n";
        if (!out) {
            std::cerr << "write failed: " << output << "\n";
            return 1;
        }
    } catch (epddl::EPDDLException& e) {
        std::cerr << e.what() << "\n";
        return 1;
    } catch (std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
