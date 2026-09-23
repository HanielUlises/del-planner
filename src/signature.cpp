#include "signature.hpp"

#include "bisimulation.hpp"
#include "knowledge_relaxation.hpp"
#include "product_update.hpp"
#include "selection_policy.hpp"
#include "symmetry.hpp"

#include <algorithm>
#include <array>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace {

struct Homology {
    std::size_t vertices{0}, edges{0}, triangles{0};
    std::size_t beta0{0};
    long long   beta1{-1};   // -1: skipped (complex too large)
};

// 2-skeleton of the simplicial complex of an epistemic state: a vertex per
// (agent, class of identical accessibility rows), a facet per world spanning its
// agents' vertices. Betti numbers over GF(2).
Homology homology(const EpistemicState& s) {
    Homology h;
    const std::uint32_t nw = s.num_worlds, na = s.num_agents;

    // Contracted states intern successor sets by content, so equal rows of one
    // agent share a set id and (agent, set) identifies a vertex.
    std::vector<std::uint32_t> vertex(std::size_t(na) * nw);
    for (AgentIdx ag = 0; ag < na; ++ag) {
        std::unordered_map<std::uint32_t, std::uint32_t> ids;
        for (WorldIdx w = 0; w < nw; ++w) {
            auto [it, fresh] = ids.try_emplace(s.succ_set(ag, w), static_cast<std::uint32_t>(h.vertices));
            if (fresh) ++h.vertices;
            vertex[std::size_t(ag) * nw + w] = it->second;
        }
    }

    const auto key2 = [](std::uint64_t a, std::uint64_t b) { return a < b ? (a << 32) | b : (b << 32) | a; };
    std::unordered_map<std::uint64_t, std::uint32_t> edge_id;
    std::vector<std::array<std::uint32_t, 3>> tri_edges;
    std::unordered_set<std::uint64_t> tri_seen;

    std::vector<std::uint32_t> parent(h.vertices);
    std::iota(parent.begin(), parent.end(), 0);
    const auto find = [&](std::uint32_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };

    const std::size_t tri_budget = 2'000'000;
    bool too_large = std::size_t(nw) * na * na * na / 6 > tri_budget * 8;

    for (WorldIdx w = 0; w < nw; ++w) {
        for (AgentIdx i = 0; i < na; ++i)
            for (AgentIdx j = i + 1; j < na; ++j) {
                const std::uint32_t a = vertex[std::size_t(i) * nw + w], b = vertex[std::size_t(j) * nw + w];
                edge_id.emplace(key2(a, b), static_cast<std::uint32_t>(edge_id.size()));
                parent[find(a)] = find(b);
            }
        if (too_large) continue;
        for (AgentIdx i = 0; i < na; ++i)
            for (AgentIdx j = i + 1; j < na; ++j)
                for (AgentIdx k = j + 1; k < na; ++k) {
                    std::array<std::uint32_t, 3> v{vertex[std::size_t(i) * nw + w],
                                                   vertex[std::size_t(j) * nw + w],
                                                   vertex[std::size_t(k) * nw + w]};
                    std::sort(v.begin(), v.end());
                    const std::uint64_t tk = bits::mix64((std::uint64_t(v[0]) << 42) ^ (std::uint64_t(v[1]) << 21) ^ v[2]);
                    if (!tri_seen.insert(tk).second) continue;
                    tri_edges.push_back({edge_id.at(key2(v[0], v[1])), edge_id.at(key2(v[0], v[2])),
                                         edge_id.at(key2(v[1], v[2]))});
                    if (tri_edges.size() > tri_budget) { too_large = true; break; }
                }
    }

    h.edges     = edge_id.size();
    h.triangles = tri_edges.size();
    for (std::uint32_t v = 0; v < h.vertices; ++v) h.beta0 += find(v) == v;
    if (too_large) return h;

    // rank ∂2 by column reduction with pivots on the lowest edge id.
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> pivot;
    std::size_t rank2 = 0;
    for (const auto& t : tri_edges) {
        std::vector<std::uint32_t> col(t.begin(), t.end());
        std::sort(col.begin(), col.end());
        while (!col.empty()) {
            auto it = pivot.find(col.back());
            if (it == pivot.end()) break;
            std::vector<std::uint32_t> x;
            std::set_symmetric_difference(col.begin(), col.end(), it->second.begin(), it->second.end(),
                                          std::back_inserter(x));
            col.swap(x);
        }
        if (!col.empty()) { pivot.emplace(col.back(), std::move(col)); ++rank2; }
    }
    const long long rank1 = static_cast<long long>(h.vertices) - static_cast<long long>(h.beta0);
    h.beta1 = static_cast<long long>(h.edges) - rank1 - static_cast<long long>(rank2);
    return h;
}

} // namespace

void print_signature(const PlanningTask& task, std::ostream& out) {
    const EpistemicState init = bisim_contract(task.init);
    const std::uint32_t rounds = last_refinement_rounds();
    const Homology topo = homology(init);

    const KnowledgeRelaxationHeuristic kadd(task);
    const auto relax = kadd.analyse(init);

    // One-step model growth after contraction.
    double growth_max = 0, growth_sum = 0;
    std::size_t applicable = 0;
    const auto cap = make_world_cap_policy(true);
    for (const Action& a : task.actions) {
        if (!a.applicable(init)) continue;
        auto r = product_update(init, a, task.frame_guard(), cap);
        if (!r) continue;
        const double g = double(bisim_contract(std::move(*r)).num_worlds) / double(std::max<std::uint32_t>(1, init.num_worlds));
        growth_max = std::max(growth_max, g);
        growth_sum += g;
        ++applicable;
    }

    std::size_t sym_components = 0, sym_largest = 0;
    if (task.symmetry) {
        const StateSymmetry st = stabiliser(*task.symmetry, init);
        if (!st.trivial)
            for (std::size_t c = 0; c + 1 < st.begin.size(); ++c) {
                const std::size_t size = st.begin[c + 1] - st.begin[c];
                sym_components += size > 1;
                sym_largest = std::max(sym_largest, size);
            }
    }

    const TaskFeatures f = TaskFeatures::extract(task);
    out << "{";
    for (auto& n : TaskFeatures::names()) out << "\"" << n << "\": " << *f.lookup(n) << ", ";
    out << "\"contracted_worlds\": " << init.num_worlds
        << ", \"bisim_rounds\": " << rounds
        << ", \"simplicial_vertices\": " << topo.vertices
        << ", \"simplicial_edges\": " << topo.edges
        << ", \"simplicial_triangles\": " << topo.triangles
        << ", \"beta0\": " << topo.beta0
        << ", \"beta1\": " << topo.beta1
        << ", \"relax_rounds\": " << relax.rounds
        << ", \"relax_depth\": " << relax.depth
        << ", \"relax_dead_goals\": " << relax.dead_goals
        << ", \"relaxed_plan\": " << relax.relaxed_plan
        << ", \"relax_facts\": " << relax.facts
        << ", \"relax_operators\": " << relax.operators
        << ", \"applicable_init\": " << applicable
        << ", \"growth_max\": " << growth_max
        << ", \"growth_mean\": " << (applicable ? growth_sum / double(applicable) : 0.0)
        << ", \"symmetry_swaps\": " << (task.symmetry ? task.symmetry->swaps.size() : 0)
        << ", \"stabiliser_components\": " << sym_components
        << ", \"stabiliser_largest\": " << sym_largest
        << "}\n";
}
