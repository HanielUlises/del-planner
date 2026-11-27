#include "symmetry.hpp"

#include <functional>

#include "bisimulation.hpp"
#include "parallel.hpp"

#include <algorithm>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_map>

namespace {

std::vector<std::string> split(const std::string& name) {
    std::vector<std::string> out(1);
    for (char c : name) {
        if (c == '_') out.emplace_back();
        else          out.back().push_back(c);
    }
    return out;
}

// Renames every token equal to agent a or b, swapping them. Nullopt if the name
// mentions neither.
std::optional<std::string> swap_tokens(const std::string& name,
                                       const std::string& a, const std::string& b) {
    auto tokens = split(name);
    bool touched = false;
    for (auto& t : tokens) {
        if (t == a)      { t = b; touched = true; }
        else if (t == b) { t = a; touched = true; }
    }
    if (!touched) return std::nullopt;
    std::string out = tokens[0];
    for (std::size_t i = 1; i < tokens.size(); ++i) out += '_' + tokens[i];
    return out;
}

// Compares formulas and actions under one swap. Formulas are hash-consed DAGs
// with heavy sharing, so hashes and verdicts are memoised by formula id: without
// that, every level of a comparison rehashed its whole subtree and shared
// subformulas were walked once per path to them.
class Matcher {
public:
    explicit Matcher(const AgentSymmetry::Swap& s) : s_(s) {}

    // Exact test that renaming f by the swap yields g, up to reordering of ∧ / ∨.
    bool equal(const Formula& f, const Formula& g) {
        const std::uint64_t key = (std::uint64_t(f.id) << 32) | g.id;
        if (auto it = eq_.find(key); it != eq_.end()) return it->second;
        const bool r = compare(f, g);
        eq_.emplace(key, r);
        return r;
    }

    bool maps_action(const Action& x, const Action& y) {
        if (x.events.size() != y.events.size() ||
            x.designated_events != y.designated_events)
            return false;

        for (std::size_t e = 0; e < x.events.size(); ++e) {
            const Event& ex = x.events[e];
            const Event& ey = y.events[e];
            if (!equal(*ex.precondition, *ey.precondition) ||
                !equal_posts(ex.post_true, ey.post_true) ||
                !equal_posts(ex.post_false, ey.post_false))
                return false;
        }

        const std::size_t na = std::max(x.obs_cases.size(), y.obs_cases.size());
        static const std::vector<ObsCase> none;
        for (AgentIdx i = 0; i < na; ++i) {
            const AgentIdx j = i == s_.a ? s_.b : i == s_.b ? s_.a : i;
            const auto& cx = i < x.obs_cases.size() ? x.obs_cases[i] : none;
            const auto& cy = j < y.obs_cases.size() ? y.obs_cases[j] : none;
            if (cx.size() != cy.size()) return false;
            for (std::size_t c = 0; c < cx.size(); ++c)
                if (cx[c].relation_bits != cy[c].relation_bits ||
                    !equal(*cx[c].condition, *cy[c].condition))
                    return false;
        }
        return true;
    }

private:
    const AgentSymmetry::Swap& s_;
    std::unordered_map<std::uint64_t, bool>          eq_;
    std::unordered_map<std::uint32_t, std::uint64_t> renamed_, plain_;

    AgentIdx ag(AgentIdx x, bool renamed) const {
        return !renamed ? x : x == s_.a ? s_.b : x == s_.b ? s_.a : x;
    }

    // Structural hash invariant under reordering of ∧ / ∨ children, of f
    // renamed by the swap or of f itself.
    std::uint64_t hash(const Formula& f, bool renamed) {
        auto& memo = renamed ? renamed_ : plain_;
        if (auto it = memo.find(f.id); it != memo.end()) return it->second;

        std::uint64_t h = bits::mix64(static_cast<std::uint64_t>(f.kind) + 1);
        switch (f.kind) {
            case FormulaKind::Atom:
                h = bits::mix64(h ^ (renamed ? s_.atom[f.atom] : f.atom));
                break;
            case FormulaKind::Belief:
            case FormulaKind::Kw:
                h = bits::mix64(h ^ ag(f.agent, renamed));
                h = bits::mix64(h ^ hash(*f.children[0], renamed));
                break;
            case FormulaKind::Common: {
                std::uint64_t g = 0;
                for (AgentIdx x : f.group) g += bits::mix64(ag(x, renamed) + 0x51);
                h = bits::mix64(h ^ g);
                h = bits::mix64(h ^ hash(*f.children[0], renamed));
                break;
            }
            case FormulaKind::Not:
                h = bits::mix64(h ^ hash(*f.children[0], renamed));
                break;
            case FormulaKind::And:
            case FormulaKind::Or: {
                std::uint64_t sum = 0;
                for (auto& c : f.children) sum += hash(*c, renamed);
                h = bits::mix64(h ^ sum);
                break;
            }
            default:
                break;
        }
        memo.emplace(f.id, h);
        return h;
    }

    bool compare(const Formula& f, const Formula& g) {
        if (f.kind != g.kind) return false;
        switch (f.kind) {
            case FormulaKind::Top:
            case FormulaKind::Bot:
                return true;
            case FormulaKind::Atom:
                return s_.atom[f.atom] == g.atom;
            case FormulaKind::Not:
                return equal(*f.children[0], *g.children[0]);
            case FormulaKind::Belief:
            case FormulaKind::Kw:
                return ag(f.agent, true) == g.agent && equal(*f.children[0], *g.children[0]);
            case FormulaKind::Common: {
                std::vector<AgentIdx> x;
                for (AgentIdx a : f.group) x.push_back(ag(a, true));
                std::vector<AgentIdx> y = g.group;
                std::sort(x.begin(), x.end());
                std::sort(y.begin(), y.end());
                return x == y && equal(*f.children[0], *g.children[0]);
            }
            case FormulaKind::And:
            case FormulaKind::Or: {
                if (f.children.size() != g.children.size()) return false;
                // Pair children by hash; a collision can only cause a false negative.
                const std::size_t n = f.children.size();
                std::vector<std::pair<std::uint64_t, const Formula*>> fc(n), gc(n);
                for (std::size_t i = 0; i < n; ++i) {
                    fc[i] = {hash(*f.children[i], true), f.children[i].get()};
                    gc[i] = {hash(*g.children[i], false), g.children[i].get()};
                }
                std::sort(fc.begin(), fc.end());
                std::sort(gc.begin(), gc.end());
                for (std::size_t i = 0; i < n; ++i)
                    if (fc[i].first != gc[i].first || !equal(*fc[i].second, *gc[i].second))
                        return false;
                return true;
            }
        }
        return false;
    }

    bool equal_posts(const std::unordered_map<AtomIdx, FormulaPtr>& x,
                     const std::unordered_map<AtomIdx, FormulaPtr>& y) {
        if (x.size() != y.size()) return false;
        for (const auto& [atom, cond] : x) {
            auto it = y.find(s_.atom[atom]);
            if (it == y.end() || !equal(*cond, *it->second)) return false;
        }
        return true;
    }
};

// The state with agents a ↔ b and their atoms swapped.
EpistemicState rename(const EpistemicState& s, const AgentSymmetry::Swap& sw) {
    EpistemicState out;
    out.allocate(s.num_worlds, s.num_atoms, s.num_agents);

    for (WorldIdx w = 0; w < s.num_worlds; ++w) {
        auto dst = out.val(w);
        bits::for_each(s.val(w), [&](std::uint32_t p) { bits::set(dst, sw.atom[p]); });
    }
    out.set_begin = s.set_begin;
    out.members   = s.members;
    for (AgentIdx ag = 0; ag < s.num_agents; ++ag) {
        const AgentIdx to = ag == sw.a ? sw.b : ag == sw.b ? sw.a : ag;
        std::copy_n(s.set_of.begin() + std::size_t(ag) * s.num_worlds, s.num_worlds,
                    out.set_of.begin() + std::size_t(to) * s.num_worlds);
    }
    bits::copy_from(out.designated_bits(), s.designated_bits());
    out.invalidate();
    return out;
}

} // namespace

AgentSymmetry AgentSymmetry::detect(const PlanningTask& task, Deadline deadline) {
    AgentSymmetry sym;
    const std::size_t na = task.num_agents();

    sym.action_agents.resize(task.num_actions());
    for (ActionIdx x = 0; x < task.num_actions(); ++x)
        for (const auto& t : split(task.actions[x].name)) {
            auto it = task.agent_index.find(t);
            if (it == task.agent_index.end()) continue;
            auto& v = sym.action_agents[x];
            if (std::find(v.begin(), v.end(), it->second) == v.end())
                v.push_back(it->second);
        }

    const Fingerprint init_fp = bisim_contract(task.init).fingerprint();

    std::vector<std::pair<AgentIdx, AgentIdx>> pairs;
    for (AgentIdx a = 0; a < na; ++a)
        for (AgentIdx b = a + 1; b < na; ++b) pairs.emplace_back(a, b);

    // Candidate swaps are independent; large models test them in parallel.
    std::vector<std::optional<Swap>> found(pairs.size());
    // Each swap is verified on its own, so stopping early keeps only swaps that
    // are true symmetries: pruning by fewer of them is still sound.
    const auto test = [&](std::size_t k) {
        if (expired(deadline)) return;
        const auto [a, b] = pairs[k];
        const auto& an = task.agent_names[a];
        const auto& bn = task.agent_names[b];

        Swap s{a, b, {}, {}};

        s.atom.resize(task.num_atoms());
        for (AtomIdx p = 0; p < task.num_atoms(); ++p) {
            s.atom[p] = p;
            if (auto r = swap_tokens(task.atom_names[p], an, bn)) {
                auto it = task.atom_index.find(*r);
                if (it == task.atom_index.end()) return;
                s.atom[p] = it->second;
            }
        }

        s.action.resize(task.num_actions());
        for (ActionIdx x = 0; x < task.num_actions(); ++x) {
            s.action[x] = x;
            if (auto r = swap_tokens(task.actions[x].name, an, bn)) {
                auto it = task.action_index.find(*r);
                if (it == task.action_index.end()) return;
                s.action[x] = it->second;
            }
        }

        Matcher m(s);
        if (task.goal && !m.equal(*task.goal, *task.goal)) return;
        for (ActionIdx x = 0; x < task.num_actions(); ++x) {
            if (expired(deadline)) return;
            if (!m.maps_action(task.actions[x], task.actions[s.action[x]])) return;
        }
        if (bisim_contract(rename(task.init, s)).fingerprint() != init_fp) return;

        found[k] = std::move(s);
    };
    if (std::size_t(task.init.num_worlds) * pairs.size() >= 4096)
        par::for_each_index(pairs.size(), test);
    else
        for (std::size_t k = 0; k < pairs.size(); ++k) test(k);

    for (auto& f : found)
        if (f) sym.swaps.push_back(std::move(*f));
    return sym;
}

namespace {
// Components of the swap graph spanned by `fixing`.
StateSymmetry components(std::uint32_t na, const std::vector<const AgentSymmetry::Swap*>& fixing) {
    StateSymmetry st;
    std::vector<std::uint32_t> parent(na);
    std::iota(parent.begin(), parent.end(), 0);
    const auto find = [&](std::uint32_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    for (const auto* sw : fixing) {
        parent[find(sw->a)] = find(sw->b);
        st.trivial = false;
    }
    if (st.trivial) return st;

    // Number components by smallest member, list members ascending.
    st.component.assign(na, UINT32_MAX);
    std::vector<std::uint32_t> id_of_root(na, UINT32_MAX);
    std::uint32_t n = 0;
    for (AgentIdx ag = 0; ag < na; ++ag) {
        const std::uint32_t r = find(ag);
        if (id_of_root[r] == UINT32_MAX) id_of_root[r] = n++;
        st.component[ag] = id_of_root[r];
    }
    st.begin.assign(n + 1, 0);
    for (AgentIdx ag = 0; ag < na; ++ag) ++st.begin[st.component[ag] + 1];
    for (std::uint32_t c = 0; c < n; ++c) st.begin[c + 1] += st.begin[c];
    st.members.resize(na);
    std::vector<std::uint32_t> fill(st.begin.begin(), st.begin.end() - 1);
    for (AgentIdx ag = 0; ag < na; ++ag) st.members[fill[st.component[ag]]++] = ag;
    return st;
}
} // namespace

StateSymmetry stabiliser(const AgentSymmetry& sym, const EpistemicState& s) {
    const std::uint32_t na = s.num_agents;

    std::vector<std::uint32_t> parent(na);
    std::iota(parent.begin(), parent.end(), 0);
    const auto find = [&](std::uint32_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };

    // Necessary conditions, cheap to check: a swap that fixes s preserves each
    // agent's multiset of row sizes and the column count of every atom it moves.
    std::vector<std::uint64_t> row_sig(na, 0);
    for (AgentIdx ag = 0; ag < na; ++ag)
        for (WorldIdx w = 0; w < s.num_worlds; ++w)
            row_sig[ag] += bits::mix64(s.succ(ag, w).size() + 1);

    std::vector<std::uint32_t> column(s.num_atoms, 0);
    for (WorldIdx w = 0; w < s.num_worlds; ++w)
        bits::for_each(s.val(w), [&](std::uint32_t p) { ++column[p]; });

    std::vector<const AgentSymmetry::Swap*> cands;
    for (const auto& sw : sym.swaps) {
        if (row_sig[sw.a] != row_sig[sw.b]) continue;
        bool ok = true;
        for (AtomIdx p = 0; p < s.num_atoms && ok; ++p)
            ok = column[p] == column[sw.atom[p]];
        if (ok) cands.push_back(&sw);
    }

    const Fingerprint fp = s.fingerprint();
    std::vector<char> fixes(cands.size(), 0);
    const auto test = [&](std::size_t k) {
        fixes[k] = bisim_contract(rename(s, *cands[k])).fingerprint() == fp;
    };
    if (par::threads() > 1 && std::size_t(s.num_worlds) * cands.size() >= 4096) {
        par::for_each_index(cands.size(), test);
    } else {
        // Serial: skip swaps whose agents are already connected.
        for (std::size_t k = 0; k < cands.size(); ++k)
            if (find(cands[k]->a) != find(cands[k]->b)) {
                test(k);
                if (fixes[k]) parent[find(cands[k]->a)] = find(cands[k]->b);
            }
    }
    std::vector<const AgentSymmetry::Swap*> fixing;
    for (std::size_t k = 0; k < cands.size(); ++k) if (fixes[k]) fixing.push_back(cands[k]);
    return components(na, fixing);
}

StateSymmetry stabiliser_by(const AgentSymmetry& sym, std::uint32_t num_agents,
                            const std::function<bool(const AgentSymmetry::Swap&)>& fixes) {
    std::vector<std::uint32_t> parent(num_agents);
    std::iota(parent.begin(), parent.end(), 0);
    const auto find = [&](std::uint32_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    std::vector<const AgentSymmetry::Swap*> fixing;
    for (const auto& sw : sym.swaps) {
        if (find(sw.a) == find(sw.b)) continue;       // already connected
        if (!fixes(sw)) continue;
        parent[find(sw.a)] = find(sw.b);
        fixing.push_back(&sw);
    }
    return components(num_agents, fixing);
}

bool StateSymmetry::keep(const AgentSymmetry& sym, ActionIdx action) const {
    if (trivial) return true;
    thread_local std::vector<std::uint32_t> used;
    used.assign(begin.size(), 0);
    for (AgentIdx ag : sym.action_agents[action]) {
        const std::uint32_t c = component[ag];
        if (begin[c + 1] - begin[c] < 2) continue;
        if (members[begin[c] + used[c]++] != ag) return false;
    }
    return true;
}
