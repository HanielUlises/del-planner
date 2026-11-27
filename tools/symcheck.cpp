// Replays a linear plan with the symbolic engine and, in the same process, with
// the explicit planner (product_update + bisim_contract), and compares them.
//
//   symcheck <task.json> <plan.json> [--enum N] [--no-explicit] [--hidden-bottom]
//
// At every step: applicability, number of worlds and designated worlds; when
// |W| <= N, kappa(explicit_state(S)) == kappa(s) — equality of canonical forms,
// i.e. exact bisimilarity (Theorem canon). One JSON line per step + a summary.
#include "symdel.hpp"
#include "bisimulation.hpp"
#include "parser.hpp"
#include "product_update.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace {
std::vector<std::string> read_plan(const char* p) {
    std::ifstream f(p); std::stringstream ss; ss << f.rdbuf(); const std::string t = ss.str();
    std::vector<std::string> out; std::size_t i = 0;
    while ((i = t.find('"', i)) != std::string::npos) {
        const auto j = t.find('"', i + 1);
        out.push_back(t.substr(i + 1, j - i - 1)); i = j + 1;
    }
    return out;
}
double secs(std::chrono::steady_clock::time_point a) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - a).count();
}
}

int main(int argc, char** argv) {
    if (argc < 3) { std::cerr << "usage: symcheck task.json plan.json [--enum N] [--no-explicit] [--hidden-bottom]\n"; return 2; }
    std::size_t enum_limit = 3000;
    bool run_explicit = true, htop = true;
    const char* plank_init = nullptr;
    for (int i = 3; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--enum")) enum_limit = std::stoul(argv[++i]);
        else if (!std::strcmp(argv[i], "--no-explicit")) run_explicit = false;
        else if (!std::strcmp(argv[i], "--hidden-bottom")) htop = false;
        else if (!std::strcmp(argv[i], "--plank-init")) plank_init = argv[++i];
    }
    std::cerr.setstate(std::ios::failbit);          // silence the parser
    const PlanningTask t = load_task(argv[1]);
    std::cerr.clear();
    if (t.repair_seriality()) { std::cout << "{\"summary\":true,\"error\":\"seriality repair unsupported\"}\n"; return 1; }
    const auto plan = std::strcmp(argv[2], "-") ? read_plan(argv[2]) : std::vector<std::string>{};
    if (t.theory && t.init.num_worlds == 0) run_explicit = false;   // not materialised

    sym::Engine eng(t, htop);
    double tsym = 0, texp = 0;
    auto t0 = std::chrono::steady_clock::now();
    sym::SymState S = eng.contract(t.theory ? eng.from_theory(*t.theory) : eng.from_explicit(t.init));
    tsym += secs(t0);
    if (plank_init) {
        std::cerr.setstate(std::ios::failbit);
        const PlanningTask p = load_task(plank_init);
        std::cerr.clear();
        const EpistemicState kp = bisim_contract(p.init);
        const bool mat_ok = t.init.num_worlds == 0 || bisim_contract(t.init) == kp;
        const bool sym_ok = eng.worlds(S) <= double(enum_limit) ? bisim_contract(eng.explicit_state(S)) == kp
                                                                 : eng.worlds(S) == kp.num_worlds;
        std::printf("{\"init_check\":true,\"plank_worlds\":%u,\"plank_kappa_worlds\":%u,\"sym_worlds\":%.0f,"
                    "\"materialised\":%s,\"mat_equal\":%s,\"sym_equal\":%s}\n",
                    p.init.num_worlds, kp.num_worlds, eng.worlds(S), t.init.num_worlds ? "true" : "false",
                    mat_ok ? "true" : "false", sym_ok ? "true" : "false");
        if (!mat_ok || !sym_ok) return 1;
    }
    EpistemicState s;
    if (run_explicit) { t0 = std::chrono::steady_clock::now(); s = bisim_contract(t.init); texp += secs(t0); }

    bool ok = true; std::size_t iso = 0, max_nodes = 0, max_hidden = 0; double max_worlds = 0;
    for (std::size_t k = 0; k <= plan.size(); ++k) {
        double ts = 0, te = 0;
        if (k > 0) {
            const Action* a = nullptr;
            for (const Action& x : t.actions) if (x.name == plan[k - 1]) a = &x;
            if (!a) { std::printf("{\"step\":%zu,\"error\":\"unknown action\"}\n", k); ok = false; break; }
            const bool app = eng.applicable(S, *a);
            if (run_explicit && app != a->applicable(s)) {
                std::printf("{\"step\":%zu,\"error\":\"applicability differs\"}\n", k); ok = false; break;
            }
            if (!app) { std::printf("{\"step\":%zu,\"error\":\"not applicable\"}\n", k); ok = false; break; }
            t0 = std::chrono::steady_clock::now();
            sym::SymState up = eng.update(S, *a);
            S = eng.contract(up);
            ts = secs(t0); tsym += ts;
            if (run_explicit) {
                t0 = std::chrono::steady_clock::now();
                s = bisim_contract(*product_update(s, *a, t.seriality()));
                te = secs(t0); texp += te;
            }
        }
        const double W = eng.worlds(S), D = eng.designated(S);
        const std::size_t nodes = eng.nodes(S);
        max_nodes = std::max(max_nodes, nodes); max_hidden = std::max(max_hidden, S.hid.size());
        max_worlds = std::max(max_worlds, W);
        std::printf("{\"step\":%zu,\"worlds\":%.0f,\"des\":%.0f,\"hidden\":%zu,\"nodes\":%zu,\"t_sym\":%.6f",
                    k, W, D, S.hid.size(), nodes, ts);
        if (run_explicit) {
            bool good = W == s.num_worlds && D == double(s.num_designated());
            std::printf(",\"exp_worlds\":%u,\"t_exp\":%.6f", s.num_worlds, te);
            if (good && W <= double(enum_limit)) {
                good = bisim_contract(eng.explicit_state(S)) == s;
                ++iso;
                std::printf(",\"iso\":%s", good ? "true" : "false");
            }
            std::printf(",\"ok\":%s", good ? "true" : "false");
            ok &= good;
        }
        std::printf("}\n");
        std::fflush(stdout);
        if (!ok) break;
    }
    const bool goal = ok && eng.holds(S, *t.goal);
    const bool goal_exp = run_explicit && ok ? s.satisfies(*t.goal) : goal;
    std::printf("{\"summary\":true,\"ok\":%s,\"goal\":%s,\"goal_explicit\":%s,\"steps\":%zu,\"iso_checked\":%zu,"
                "\"max_worlds\":%.0f,\"max_hidden\":%zu,\"max_nodes\":%zu,\"bdd_nodes_total\":%zu,"
                "\"t_sym\":%.4f,\"t_exp\":%.4f,\"t_reach\":%.4f,\"t_fixpoint\":%.4f,\"t_quotient\":%.4f,"
                "\"fixpoint_runs\":%zu,\"fixpoint_rounds\":%zu}\n",
                ok && goal == goal_exp ? "true" : "false", goal ? "true" : "false", goal_exp ? "true" : "false",
                plan.size(), iso, max_worlds, max_hidden, max_nodes, eng.mgr().size(), tsym, texp,
                eng.stats.t_reach, eng.stats.t_fixpoint, eng.stats.t_quotient,
                eng.stats.fixpoint_runs, eng.stats.fixpoint_rounds);
    return ok ? 0 : 1;
}
