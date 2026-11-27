#include "symdel.hpp"

#include <algorithm>
#include <functional>
#include <chrono>
#include <optional>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>

namespace sym {

using bdd::kFalse;
using bdd::kTrue;

namespace {
// Disjunction of many BDDs as a balanced tree: a running OR of n cubes costs
// O(n) per step, the tree O(log n) levels of merges between similar sizes.
Ref or_all(bdd::Manager& m, std::vector<Ref> v) {
    if (v.empty()) return kFalse;
    while (v.size() > 1) {
        std::size_t k = 0;
        for (std::size_t i = 0; i + 1 < v.size(); i += 2) v[k++] = m.bor(v[i], v[i + 1]);
        if (v.size() % 2) v[k++] = v.back();
        v.resize(k);
    }
    return v[0];
}

std::uint32_t bits_for(std::size_t n) {          // ceil(log2 n), 0 for n <= 1
    std::uint32_t b = 0;
    while ((std::size_t(1) << b) < n) ++b;
    return b;
}
}

Engine::Engine(const PlanningTask& t, bool hidden_on_top)
    : t_(t),
      na_(static_cast<std::uint32_t>(t.num_atoms())),
      ng_(static_cast<std::uint32_t>(t.num_agents())),
      htop_(hidden_on_top),
      m_(3 * (static_cast<std::uint32_t>(t.num_atoms()) + kMaxHidden) +
         bits_for(t.num_agents() + 2) + static_cast<std::uint32_t>(t.num_atoms()) + kBlockBits) {
    base_ = 3 * (na_ + kMaxHidden);
    abits_ = bits_for(ng_ + 2);
    if (const char* e = std::getenv("SYM_SIGNATURE")) signature_refinement = e[0] == '1';
    if (const char* e = std::getenv("SYM_NOCERT")) minimality_certificate = !(e[0] == '1');
    XY_    = copy_map({{0, 1}});
    YX_    = copy_map({{1, 0}});
    YU_    = copy_map({{1, 2}});
    SWAP_  = copy_map({{0, 1}, {1, 0}});
    XY_UX_ = copy_map({{0, 2}, {1, 0}});
    X_U_   = copy_map({{0, 2}});
    XY_YU_ = copy_map({{0, 1}, {1, 2}});
    UX_    = copy_map({{2, 0}});
    UY_    = copy_map({{2, 1}});
    XYPAIR_.assign(m_.num_vars(), 0);
    for (std::uint32_t p = 0; p < na_; ++p) XYPAIR_[av(p, 0)] = 1;
    for (std::uint32_t k = 0; k < kMaxHidden; ++k) XYPAIR_[hv(k, 0)] = 1;
    for (std::uint32_t p = na_; p-- > 0;)
        eqP_ = m_.band(eqP_, m_.biff(m_.var(av(p, 0)), m_.var(av(p, 1))));

}

std::vector<std::uint32_t> Engine::copy_map(std::initializer_list<std::pair<unsigned, unsigned>> mp) const {
    std::vector<std::uint32_t> perm(m_.num_vars());
    for (std::uint32_t v = 0; v < perm.size(); ++v) perm[v] = v;
    for (auto [from, to] : mp) {
        for (std::uint32_t p = 0; p < na_; ++p) perm[av(p, from)] = av(p, to);
        for (std::uint32_t k = 0; k < kMaxHidden; ++k) perm[hv(k, from)] = hv(k, to);
    }
    return perm;
}

Ref Engine::vars(const std::vector<std::uint32_t>& hid, unsigned c, bool atoms) {
    std::vector<std::uint32_t> vs;
    if (atoms) for (std::uint32_t p = 0; p < na_; ++p) vs.push_back(av(p, c));
    for (std::uint32_t k : hid) vs.push_back(hv(k, c));
    return m_.varset(std::move(vs));
}

Ref Engine::code(const std::vector<std::uint32_t>& slots, std::uint32_t value, unsigned c) {
    std::vector<std::pair<std::uint32_t, bool>> lits;
    for (std::size_t j = 0; j < slots.size(); ++j) lits.push_back({hv(slots[j], c), bool((value >> j) & 1u)});
    return m_.cube(std::move(lits));
}

std::size_t Engine::nodes(const SymState& s) const {
    std::vector<Ref> roots{s.theta, s.delta};
    roots.insert(roots.end(), s.om.begin(), s.om.end());
    return m_.node_count(roots);
}

// ---------------------------------------------------------------- model checking

Ref Engine::box(SymState& s, std::uint32_t ag, Ref ext) {
    // w ∈ theta with no R_ag-successor outside ext. Successors share most
    // relations with their parent, so results are kept across states, keyed by
    // the BDDs they depend on (the hidden slots in use are a function of θ).
    const BoxKey key{s.om[ag], ext, s.theta};
    if (auto it = box_memo_.find(key); it != box_memo_.end()) { ++stats.box_hits; return it->second; }
    const Ref out = m_.neg(m_.permute(ext, XY_, true));
    const Ref bad = m_.and_exists(s.om[ag], out, vars(s.hid, 1));
    const Ref r = m_.bdiff(s.theta, bad);
    box_memo_.emplace(key, r);
    return r;
}

Ref Engine::sat(SymState& s, const Formula& f) {
    if (auto it = s.memo.find(f.id); it != s.memo.end()) return it->second;
    Ref r = kFalse;
    switch (f.kind) {
    case FormulaKind::Top:  r = s.theta; break;
    case FormulaKind::Bot:  r = kFalse; break;
    case FormulaKind::Atom: r = m_.band(s.theta, m_.var(av(f.atom, 0))); break;
    case FormulaKind::Not:  r = m_.bdiff(s.theta, sat(s, *f.children[0])); break;
    case FormulaKind::And:
        r = s.theta;
        for (auto& c : f.children) r = m_.band(r, sat(s, *c));
        break;
    case FormulaKind::Or:
        r = kFalse;
        for (auto& c : f.children) r = m_.bor(r, sat(s, *c));
        break;
    case FormulaKind::Belief: r = box(s, f.agent, sat(s, *f.children[0])); break;
    case FormulaKind::Kw: {
        const Ref e = sat(s, *f.children[0]);
        r = m_.bor(box(s, f.agent, e), box(s, f.agent, m_.bdiff(s.theta, e)));
        break;
    }
    case FormulaKind::Common: {          // C_G φ = νY. ∧_{i∈G} [i](φ ∧ Y)
        const Ref e = sat(s, *f.children[0]);
        Ref y = s.theta;
        while (true) {
            Ref z = s.theta;
            for (AgentIdx g : f.group) z = m_.band(z, box(s, g, m_.band(e, y)));
            if (z == y) break;
            y = z;
        }
        r = y;
        break;
    }
    }
    s.memo.emplace(f.id, r);
    return r;
}

bool Engine::holds(SymState& s, const Formula& f) { return m_.subset(s.delta, sat(s, f)); }

bool Engine::applicable(SymState& s, const Action& a) {
    Ref ok = kFalse;
    for (EventIdx e : a.designated_events) ok = m_.bor(ok, sat(s, *a.events[e].precondition));
    return m_.subset(s.delta, ok);
}

// ---------------------------------------------------------------- construction

SymState Engine::from_explicit(const EpistemicState& e) {
    const std::uint32_t nw = e.num_worlds;
    std::map<std::vector<bits::Word>, std::vector<WorldIdx>> groups;
    for (WorldIdx w = 0; w < nw; ++w) {
        auto v = e.val(w);
        groups[std::vector<bits::Word>(v.begin(), v.end())].push_back(w);
    }
    std::size_t mult = 1;
    for (auto& [_, g] : groups) mult = std::max(mult, g.size());
    SymState s;
    for (std::uint32_t k = 0; k < bits_for(mult); ++k) s.hid.push_back(k);
    std::vector<std::uint32_t> rank(nw);
    for (auto& [_, g] : groups) for (std::size_t r = 0; r < g.size(); ++r) rank[g[r]] = static_cast<std::uint32_t>(r);

    auto world_cube = [&](WorldIdx w, unsigned c) {
        std::vector<std::pair<std::uint32_t, bool>> lits;
        for (std::uint32_t p = 0; p < na_; ++p) lits.push_back({av(p, c), e.has_atom(w, p)});
        for (std::size_t j = 0; j < s.hid.size(); ++j) lits.push_back({hv(s.hid[j], c), bool((rank[w] >> j) & 1u)});
        return m_.cube(std::move(lits));
    };
    std::vector<Ref> cx(nw), cy(nw), des;
    for (WorldIdx w = 0; w < nw; ++w) {
        cx[w] = world_cube(w, 0);
        cy[w] = world_cube(w, 1);
        if (e.is_designated(w)) des.push_back(cx[w]);
    }
    s.theta = or_all(m_, cx);
    s.delta = or_all(m_, std::move(des));
    std::vector<std::vector<Ref>> members(e.num_sets());
    for (std::uint32_t id = 0; id < e.num_sets(); ++id)
        for (WorldIdx v : e.set(id)) members[id].push_back(cy[v]);
    std::vector<Ref> setb(e.num_sets(), kFalse);
    for (std::uint32_t id = 0; id < e.num_sets(); ++id) setb[id] = or_all(m_, std::move(members[id]));
    for (AgentIdx g = 0; g < ng_; ++g) {
        std::vector<std::vector<Ref>> src(e.num_sets());
        for (WorldIdx w = 0; w < nw; ++w) src[e.succ_set(g, w)].push_back(cx[w]);
        std::vector<Ref> parts;
        for (std::uint32_t id = 0; id < e.num_sets(); ++id)
            if (!src[id].empty() && setb[id] != kFalse) parts.push_back(m_.band(or_all(m_, std::move(src[id])), setb[id]));
        s.om.push_back(or_all(m_, std::move(parts)));
    }
    return s;
}

Ref Engine::prop(const Formula& f, unsigned c) {
    switch (f.kind) {
    case FormulaKind::Top:  return kTrue;
    case FormulaKind::Bot:  return kFalse;
    case FormulaKind::Atom: return m_.var(av(f.atom, c));
    case FormulaKind::Not:  return m_.neg(prop(*f.children[0], c));
    case FormulaKind::And: {
        Ref r = kTrue;
        for (auto& ch : f.children) r = m_.band(r, prop(*ch, c));
        return r;
    }
    case FormulaKind::Or: {
        Ref r = kFalse;
        for (auto& ch : f.children) r = m_.bor(r, prop(*ch, c));
        return r;
    }
    default:
        throw std::runtime_error("finitary S5 theory: formula is not propositional");
    }
}

SymState Engine::from_theory(const InitialTheory& th) {
    SymState s;
    Ref theta = kTrue;
    for (auto [p, v] : th.fixed) theta = m_.band(theta, v ? m_.var(av(p, 0)) : m_.nvar(av(p, 0)));
    for (const auto& f : th.worlds) theta = m_.band(theta, prop(*f, 0));
    Ref delta = theta;
    for (const auto& f : th.designated) delta = m_.band(delta, prop(*f, 0));
    if (theta == kFalse) throw std::runtime_error("finitary S5 theory: no world satisfies the theory");
    if (delta == kFalse) throw std::runtime_error("finitary S5 theory: no designated world");
    s.theta = theta;
    s.delta = delta;
    const Ref tt = m_.band(theta, m_.permute(theta, XY_, true));
    for (AgentIdx g = 0; g < ng_; ++g) {
        Ref agree = tt;
        if (g < th.observes.size())
            for (const auto& f : th.observes[g]) agree = m_.band(agree, m_.biff(prop(*f, 0), prop(*f, 1)));
        s.om.push_back(agree);
    }
    return s;
}

// ---------------------------------------------------------------- product update

SymState Engine::update(SymState& s, const Action& a) {
    const std::uint32_t ne = static_cast<std::uint32_t>(a.events.size());
    const std::uint32_t base = static_cast<std::uint32_t>(s.hid.size());
    for (std::uint32_t j = 0; j < base; ++j)
        if (s.hid[j] != j) throw std::logic_error("update expects a normalised state");

    // Observability for the whole state, as in plank: the first case whose
    // condition holds at every designated world, else the first declared type.
    std::vector<const ObsCase*> obs(ng_, &a.default_obs);
    for (AgentIdx g = 0; g < ng_ && g < a.obs_cases.size(); ++g)
        for (const ObsCase& c : a.obs_cases[g])
            if (m_.subset(s.delta, sat(s, *c.condition))) { obs[g] = &c; break; }

    std::vector<Ref> pre(ne);
    for (EventIdx e = 0; e < ne; ++e) pre[e] = sat(s, *a.events[e].precondition);
    struct Guard { EventIdx e; AtomIdx p; bool val; Ref ext; };
    std::vector<Guard> guards;
    std::vector<AtomIdx> changed;
    for (EventIdx e = 0; e < ne; ++e) {
        for (auto& [p, f] : a.events[e].post_true)  { guards.push_back({e, p, true,  sat(s, *f)}); changed.push_back(p); }
        for (auto& [p, f] : a.events[e].post_false) { guards.push_back({e, p, false, sat(s, *f)}); changed.push_back(p); }
    }
    std::sort(changed.begin(), changed.end());
    changed.erase(std::unique(changed.begin(), changed.end()), changed.end());

    // Fresh slots: event code, then the old value of each changed atom.
    const std::uint32_t nb = bits_for(ne);
    if (base + nb + changed.size() > kMaxHidden) throw std::runtime_error("too many hidden slots");
    std::vector<std::uint32_t> ev, all = s.hid;
    for (std::uint32_t j = 0; j < nb; ++j) ev.push_back(base + j);
    std::vector<std::uint32_t> old(na_, 0);
    for (std::size_t j = 0; j < changed.size(); ++j) old[changed[j]] = base + nb + static_cast<std::uint32_t>(j);
    all.insert(all.end(), ev.begin(), ev.end());
    for (AtomIdx p : changed) all.push_back(old[p]);

    // L moves changed atoms to their old-value slots (x_p -> hx_o, y_p -> hy_o).
    std::vector<std::uint32_t> mv(m_.num_vars());
    for (std::uint32_t v = 0; v < mv.size(); ++v) mv[v] = v;
    for (AtomIdx p : changed) { mv[av(p, 0)] = hv(old[p], 0); mv[av(p, 1)] = hv(old[p], 1); }
    auto L = [&](Ref f) { return changed.empty() ? f : m_.permute(f, mv, false); };

    std::vector<Ref> cx(ne), cy(ne);
    for (EventIdx e = 0; e < ne; ++e) { cx[e] = code(ev, e, 0); cy[e] = code(ev, e, 1); }

    SymState n;
    n.hid = all;
    Ref worlds = kFalse;
    for (EventIdx e = 0; e < ne; ++e) worlds = m_.bor(worlds, m_.band(cx[e], L(pre[e])));
    Ref theta = m_.band(L(s.theta), worlds);
    for (AtomIdx p : changed) {
        const Ref oldp = m_.var(hv(old[p], 0));
        Ref nv = kFalse;
        for (EventIdx e = 0; e < ne; ++e) {
            Ref val = oldp;            // sets precede resets: (old ∨ T) ∧ ¬F
            for (const Guard& g : guards) if (g.e == e && g.p == p && g.val)  val = m_.bor(val, L(g.ext));
            for (const Guard& g : guards) if (g.e == e && g.p == p && !g.val) val = m_.bdiff(val, L(g.ext));
            nv = m_.bor(nv, m_.band(cx[e], val));
        }
        theta = m_.band(theta, m_.biff(m_.var(av(p, 0)), nv));
    }
    n.theta = theta;
    Ref des = kFalse;
    for (EventIdx e : a.designated_events) des = m_.bor(des, cx[e]);
    n.delta = m_.band(m_.band(L(s.delta), theta), des);
    const Ref theta_y = m_.permute(theta, XY_, true);
    const Ref tt = m_.band(theta, theta_y);
    for (AgentIdx g = 0; g < ng_; ++g) {
        Ref q = kFalse;
        for (EventIdx e = 0; e < ne; ++e) {
            Ref tg = kFalse;
            const auto row = obs[g]->event_row(e);
            for (EventIdx f = 0; f < ne; ++f) if (bits::test(row, f)) tg = m_.bor(tg, cy[f]);
            if (tg != kFalse) q = m_.bor(q, m_.band(cx[e], tg));
        }
        n.om.push_back(m_.band(m_.band(L(s.om[g]), q), tt));
    }
    return n;
}

// ---------------------------------------------------------------- contraction

void Engine::drop_determined(SymState& q) {
    for (std::size_t j = 0; j < q.hid.size();) {
        const std::uint32_t k = q.hid[j];
        if (m_.band(m_.cofactor(q.theta, hv(k, 0), true), m_.cofactor(q.theta, hv(k, 0), false)) == kFalse) {
            const Ref hx = m_.varset({hv(k, 0)}), hxy = m_.varset({hv(k, 0), hv(k, 1)});
            q.theta = m_.exists(q.theta, hx);
            q.delta = m_.exists(q.delta, hx);
            for (Ref& o : q.om) o = m_.exists(o, hxy);
            q.hid.erase(q.hid.begin() + j);
        } else {
            ++j;
        }
    }
}

Ref Engine::bisimulation(const std::vector<std::uint32_t>& hid, Ref theta, Ref delta,
                         const std::vector<Ref>& om, Ref E_init) {
    // E is symmetric (E_0 is, and each round intersects with Forth and its
    // transpose), so E(u,x) = E(x,u), an order-preserving relabelling (copies
    // are interleaved per variable, x < y < u) instead of an ITE rebuild.
    //   Forth(x,y) = ¬∃u. R(x,u) ∧ ¬A(y,u),   A(y,u) = ∃x. R(y,x) ∧ E(x,u)
    // Computing the transpose Back directly costs two more relational products
    // per agent and measured slower than transposing Forth.
    const Ref X = vars(hid, 0), U = vars(hid, 2);
    const Ref tt = m_.band(theta, m_.permute(theta, XY_, true));
    // Any symmetric start that contains the maximal bisimulation will do.
    Ref E = E_init != kFalse ? E_init
                             : m_.band(m_.band(tt, eqP_), m_.biff(delta, m_.permute(delta, XY_, true)));
    std::vector<Ref> om_sw(ng_), om_xu(ng_);
    for (AgentIdx g = 0; g < ng_; ++g) {
        om_sw[g] = m_.swap_adjacent(om[g], XYPAIR_);  // R(y -> x)
        om_xu[g] = m_.permute(om[g], YU_, true);      // R(x -> u)
    }
    while (true) {
        ++stats.fixpoint_rounds;
        const Ref E_xu = m_.permute(E, YU_, true);
        Ref E2 = E;
        for (AgentIdx g = 0; g < ng_ && E2 != kFalse; ++g) {
            const Ref A = m_.and_exists(om_sw[g], E_xu, X);               // A(y,u)
            const Ref bad = m_.and_exists(om_xu[g], m_.neg(A), U);        // x has an unmatched succ
            const Ref F = m_.neg(bad);
            E2 = m_.band(E2, m_.band(F, m_.swap_adjacent(F, XYPAIR_)));
        }
        if (E2 == E) break;
        E = E2;
    }
    return E;
}


SymState Engine::contract(const SymState& s) {
    ++stats.contractions;
    std::optional<PhaseTimer> phase(std::in_place, stats.t_reach);
    const Ref X = vars(s.hid, 0);
    Ref anyR = kFalse;
    for (Ref o : s.om) anyR = m_.bor(anyR, o);

    // 1. worlds reachable from delta
    Ref reach = s.delta;
    while (true) {
        const Ref img = m_.permute(m_.and_exists(reach, anyR, X), YX_, true);
        const Ref next = m_.bor(reach, img);
        if (next == reach) break;
        reach = next;
    }
    SymState r;
    r.hid = s.hid;
    r.theta = reach;
    const Ref tt = m_.band(reach, m_.permute(reach, XY_, true));
    for (AgentIdx g = 0; g < ng_; ++g) r.om.push_back(m_.band(s.om[g], tt));
    r.delta = m_.band(s.delta, reach);

    // 2. Hidden slots that the other variables determine carry no information.
    //    Once they are gone, H = ∅ means distinct worlds have distinct
    //    valuations; bisimilar worlds agree on the valuation, so the quotient
    //    is the model itself and the fixpoint is skipped.
    drop_determined(r);
    if (r.hid.empty()) { ++stats.fixpoint_skipped; return r; }

    SymState q;
    Ref E_init = kFalse;
    if (minimality_certificate) {
        // Pairs of distinct worlds with equal valuation and designation that
        // depth one does not tell apart; when there are none, the model is
        // its own quotient. Otherwise the fixpoint starts from them.
        phase.emplace(stats.t_certificate);
        const Ref tt = m_.band(r.theta, m_.permute(r.theta, XY_, true));
        const Ref same = same_world(r.hid, r.theta);
        const Ref D0 = m_.bdiff(m_.band(m_.band(tt, eqP_), m_.biff(r.delta, m_.permute(r.delta, XY_, true))), same);
        const Ref D = depth_one_pairs(r.hid, r.theta, r.delta, r.om, D0);
        if (D == kFalse) {
            ++stats.certified;
            phase.emplace(stats.t_quotient);
            return finish_quotient(std::move(r));
        }
        E_init = m_.bor(same, D);
    }
    ++stats.fixpoint_runs;
    if (signature_refinement) {
        phase.emplace(stats.t_fixpoint);
        q = refine_quotient(r);
        phase.emplace(stats.t_quotient);
    } else {
        q = pairwise_quotient(r, phase, E_init);
    }
    return finish_quotient(std::move(q));
}

// Minimality certificate. After projection every hidden bit separates some
// pair of worlds with equal valuations. Two worlds that differ in their
// valuation, their designation, or, for some agent i, in the set of pairs
// (valuation, designated?) of their R_i-successors are not bisimilar; if
// every pair of distinct worlds with the same valuation and designation
// differs in that way, the maximal bisimulation is the identity and the
// model is already its own quotient. The successor sets quantify the
// successors' hidden bits away first, so the comparison is between functions
// of a world and a valuation (copies x, y and the atoms of u), not between
// worlds of three copies. Returns false when it cannot decide; the fixpoint
// then runs as before.
Ref Engine::depth_one_pairs(const std::vector<std::uint32_t>& hid, Ref theta, Ref delta,
                            const std::vector<Ref>& om, Ref D) {
    if (D == kFalse) return D;
    const Ref YH = vars(hid, 1, false);
    std::vector<std::uint32_t> up;
    for (std::uint32_t p = 0; p < na_; ++p) up.push_back(av(p, 2));
    const Ref UP = m_.varset(up);
    const Ref delta_y = m_.permute(delta, XY_, true);
    const Ref theta_y = m_.permute(theta, XY_, true);
    for (AgentIdx g = 0; g < ng_; ++g)
        for (int des = 1; des >= 0; --des) {
            const Ref S = m_.and_exists(om[g], des ? delta_y : m_.bdiff(theta_y, delta_y), YH);   // S(x, y_P)
            const Ref Sxu = m_.permute(S, YU_, true);                  // S(x, u_P)
            const Ref Syu = m_.permute(S, XY_YU_, true);               // S(y, u_P)
            D = m_.bdiff(D, m_.and_exists(D, m_.bxor(Sxu, Syu), UP));
            if (D == kFalse) return D;
        }
    return D;
}

Ref Engine::same_world(const std::vector<std::uint32_t>& hid, Ref theta) {
    Ref sameH = kTrue;
    for (std::uint32_t k : hid) sameH = m_.band(sameH, m_.biff(m_.var(hv(k, 0)), m_.var(hv(k, 1))));
    return m_.band(m_.band(theta, eqP_), sameH);
}

// Coarsest bisimulation by signature refinement (as in BDD-based bisimulation
// minimisation, Wimmer et al. 2006). The partition is a function L(x, l)
// from worlds to block numbers, with the block bits l below every x
// variable; a block is (valuation, l), so l need not separate valuations.
// The signature of x lists, over the bits (a, z, l) below x:
//   a = SELF, l = L(x)                 its own block (so rounds only refine)
//   a = DES                            whether it is designated
//   a = i, z = V(y), l = L(y)          the block of each R_i-successor y
// Equal signatures are equal sub-BDDs at the first node below the x levels,
// so one pass over the signature's x part numbers the new blocks. No relation
// over two copies of the worlds is built, and no product over three.
SymState Engine::refine_quotient(const SymState& r) {
    auto acode = [&](std::uint32_t v) {
        std::vector<std::pair<std::uint32_t, bool>> lits;
        for (std::uint32_t j = 0; j < abits_; ++j) lits.push_back({base_ + j, bool((v >> j) & 1u)});
        return m_.cube(std::move(lits));
    };
    auto lcube = [&](std::uint32_t id, std::uint32_t nbits) {
        std::vector<std::pair<std::uint32_t, bool>> lits;
        for (std::uint32_t j = 0; j < nbits; ++j) lits.push_back({lv(j), bool((id >> j) & 1u)});
        return m_.cube(std::move(lits));
    };
    std::vector<std::pair<std::uint32_t, bool>> zl;
    for (std::uint32_t p = 0; p < na_; ++p) zl.push_back({zv(p), false});
    const Ref zero_z = m_.cube(zl);
    for (std::uint32_t j = 0; j < kBlockBits; ++j) zl.push_back({lv(j), false});
    const Ref zero_zl = m_.cube(zl);

    // G_i(x, z, y_H) = Ω_i with the successor's valuation moved to z.
    std::vector<std::uint32_t> yz(m_.num_vars()), lz(m_.num_vars());
    for (std::uint32_t v = 0; v < yz.size(); ++v) yz[v] = lz[v] = v;
    for (std::uint32_t p = 0; p < na_; ++p) { yz[av(p, 1)] = zv(p); lz[av(p, 0)] = zv(p); }
    for (std::uint32_t k : r.hid) lz[hv(k, 0)] = hv(k, 1);
    std::vector<Ref> G(ng_);
    for (AgentIdx g = 0; g < ng_; ++g) G[g] = m_.permute(r.om[g], yz, false);
    const Ref YH = vars(r.hid, 1, false);
    const Ref des_term = m_.band(acode(ng_ + 1), m_.band(r.delta, zero_zl));
    const Ref self_code = m_.band(acode(ng_), zero_z);
    std::vector<Ref> agent_code(ng_);
    for (AgentIdx g = 0; g < ng_; ++g) agent_code[g] = acode(g);

    Ref L = m_.band(r.theta, lcube(0, kBlockBits));
    std::size_t count = 1;
    while (true) {
        ++stats.fixpoint_rounds;
        const Ref Lz = m_.permute(L, lz, false);              // L(y_H, z, l)
        Ref sig = m_.bor(des_term, m_.band(self_code, L));
        for (AgentIdx g = 0; g < ng_; ++g)
            sig = m_.bor(sig, m_.band(agent_code[g], m_.and_exists(G[g], Lz, YH)));
        std::unordered_map<Ref, std::uint32_t> ids;
        std::unordered_map<Ref, Ref> memo;
        std::function<Ref(Ref)> number = [&](Ref f) -> Ref {
            if (f == kFalse) return kFalse;
            if (f == kTrue || m_.top(f) >= base_) {
                const auto [it, fresh] = ids.try_emplace(f, static_cast<std::uint32_t>(ids.size()));
                if (ids.size() >= (std::size_t(1) << kBlockBits)) throw std::runtime_error("too many blocks");
                return lcube(it->second, kBlockBits);
            }
            if (auto it = memo.find(f); it != memo.end()) return it->second;
            const Ref l = number(m_.lo(f));
            const Ref out = m_.mk(m_.top(f), l, number(m_.hi(f)));
            memo.emplace(f, out);
            return out;
        };
        const Ref Ln = number(sig);
        if (ids.size() == count) break;          // a bijection on blocks: stable
        count = ids.size();
        L = Ln;
    }

    // Quotient as in the pairwise version, with E read off the final blocks
    // once. Block numbers as the new hidden encoding would discard the
    // structure of the old one (event codes, old values) and measured 15x
    // larger relations.
    std::vector<std::uint32_t> lvars;
    for (std::uint32_t j = 0; j < kBlockBits; ++j) lvars.push_back(lv(j));
    const Ref LV = m_.varset(lvars);
    // Bit j of x's block, β_j(x) = ∃l. L(x,l) ∧ l_j; then same block is a
    // conjunction of equivalences between interleaved copies. (A relational
    // product over l, with the block bits below both copies, costs |L|^2.)
    Ref E = m_.band(eqP_, m_.band(r.theta, m_.permute(r.theta, XY_, true)));
    for (std::uint32_t j = 0; j < bits_for(count); ++j) {
        const Ref b = m_.exists(m_.band(L, m_.var(lv(j))), LV);
        E = m_.band(E, m_.biff(b, m_.permute(b, XY_, true)));
    }
    return quotient_by(r, E);
}

SymState Engine::pairwise_quotient(const SymState& r, std::optional<PhaseTimer>& phase, Ref E_init) {
    // 3. maximal bisimulation E(x,y), a greatest fixpoint
    phase.emplace(stats.t_fixpoint);
    const Ref E = bisimulation(r.hid, r.theta, r.delta, r.om, E_init);
    phase.emplace(stats.t_quotient);
    return quotient_by(r, E);
}

SymState Engine::quotient_by(const SymState& r, Ref E) {
    const Ref Y = vars(r.hid, 1), U = vars(r.hid, 2);

    // 4. one representative per class: the least hidden assignment
    Ref lt = kFalse, eq = kTrue;                 // y_H <lex x_H
    for (std::uint32_t k : r.hid) {
        const Ref hx = m_.var(hv(k, 0)), hy = m_.var(hv(k, 1));
        lt = m_.bor(lt, m_.band(eq, m_.band(m_.neg(hy), hx)));
        eq = m_.band(eq, m_.biff(hx, hy));
    }
    const Ref rep = m_.bdiff(r.theta, m_.and_exists(E, lt, Y));
    const Ref rep_y = m_.permute(rep, XY_, true), rr = m_.band(rep, rep_y);
    const Ref E_uy = m_.permute(E, XY_YU_, true);      // E(y,u) = E(u,y), E symmetric
    SymState q;
    q.hid = r.hid;
    q.theta = rep;
    q.delta = m_.band(r.delta, rep);
    for (AgentIdx g = 0; g < ng_; ++g)
        q.om.push_back(m_.band(rr, m_.and_exists(m_.permute(r.om[g], YU_, true), E_uy, U)));
    return q;
}

SymState Engine::finish_quotient(SymState q) {
    // 5. drop hidden slots determined by the other variables on theta
    drop_determined(q);

    // 5b. re-encode the N distinct hidden vectors onto ceil(log2 N) fresh slots,
    //     lexicographically: injective on hidden vectors, hence on worlds.
    if (!q.hid.empty()) {
        const Ref types = m_.exists(q.theta, vars({}, 0, true));
        const double N = m_.count(types, xvars(q.hid, false));
        const std::uint32_t mb = bits_for(static_cast<std::size_t>(N));
        if (mb < q.hid.size() && N <= double(reencode_limit)) {
            std::vector<std::uint32_t> hxv;
            for (std::uint32_t k : q.hid) hxv.push_back(hv(k, 0));
            std::sort(hxv.begin(), hxv.end());
            std::vector<std::vector<std::uint8_t>> vecs;
            m_.enumerate(types, hxv, vecs);
            const std::uint32_t top = *std::max_element(q.hid.begin(), q.hid.end()) + 1;
            std::vector<std::uint32_t> fresh;
            for (std::uint32_t j = 0; j < mb; ++j) fresh.push_back(top + j);
            if (top + mb > kMaxHidden) throw std::runtime_error("too many hidden slots");
            auto mapping = [&](unsigned c) {
                Ref M = kFalse;
                for (std::size_t j = 0; j < vecs.size(); ++j) {
                    std::vector<std::pair<std::uint32_t, bool>> lits;
                    for (std::size_t i = 0; i < hxv.size(); ++i) lits.push_back({hxv[i] + c, bool(vecs[j][i])});
                    for (std::size_t i = 0; i < fresh.size(); ++i) lits.push_back({hv(fresh[i], c), bool((j >> i) & 1u)});
                    M = m_.bor(M, m_.cube(std::move(lits)));
                }
                return M;
            };
            const Ref Mx = mapping(0), My = mapping(1);
            const Ref HX = vars(q.hid, 0, false), HY = vars(q.hid, 1, false);
            q.theta = m_.and_exists(q.theta, Mx, HX);
            q.delta = m_.and_exists(q.delta, Mx, HX);
            for (Ref& o : q.om) o = m_.and_exists(m_.and_exists(o, Mx, HX), My, HY);
            q.hid = fresh;
        }
    }

    // 6. normalise: remaining slots move to 0..m-1 (order-preserving)
    bool moved = false;
    std::vector<std::uint32_t> nm(m_.num_vars());
    for (std::uint32_t v = 0; v < nm.size(); ++v) nm[v] = v;
    for (std::size_t j = 0; j < q.hid.size(); ++j)
        if (q.hid[j] != j) {
            moved = true;
            for (unsigned c = 0; c < 2; ++c) nm[hv(q.hid[j], c)] = hv(static_cast<std::uint32_t>(j), c);
        }
    if (moved) {
        q.theta = m_.permute(q.theta, nm, true);
        q.delta = m_.permute(q.delta, nm, true);
        for (Ref& o : q.om) o = m_.permute(o, nm, true);
    }
    for (std::size_t j = 0; j < q.hid.size(); ++j) q.hid[j] = static_cast<std::uint32_t>(j);
    return q;
}

// ---------------------------------------------------------------- memory and identity

void Engine::roots_of(const SymState& s, std::vector<Ref>& out) {
    out.push_back(s.theta);
    out.push_back(s.delta);
    out.insert(out.end(), s.om.begin(), s.om.end());
    for (const auto& [_, r] : s.memo) out.push_back(r);
}

std::size_t Engine::gc(std::vector<Ref> roots) {
    roots.push_back(eqP_);
    const std::size_t freed = m_.gc(roots);
    stats.gc_freed += freed;
    hmemo_.clear();
    box_memo_.clear();
    return freed;
}

std::pair<std::uint64_t, std::uint64_t> Engine::shash(Ref r) {
    if (hmemo_epoch_ != m_.gc_epoch()) { hmemo_.clear(); hmemo_epoch_ = m_.gc_epoch(); }
    if (r == kFalse) return {0x8F1BBCDCCA62C1D6ull, 0x5A827999ED9EBA1Full};
    if (r == kTrue)  return {0x6ED9EBA1A54FF53Aull, 0x1F83D9AB5BE0CD19ull};
    if (auto it = hmemo_.find(r); it != hmemo_.end()) return it->second;
    const auto l = shash(m_.lo(r)), h = shash(m_.hi(r));
    const std::uint64_t v = m_.top(r);
    std::pair<std::uint64_t, std::uint64_t> out{
        bits::mix64(bits::mix64(v * 0x9E3779B97F4A7C15ull ^ l.first) ^ (h.first + 0x632BE59BD9B4E019ull)),
        bits::mix64(bits::mix64((v + 0xC2B2AE3D27D4EB4Full) ^ l.second) * 0x165667B19E3779F9ull ^ h.second)};
    hmemo_.emplace(r, out);
    return out;
}

namespace {
Fingerprint combine(std::uint64_t tag, const std::vector<std::pair<std::uint64_t, std::uint64_t>>& parts) {
    Fingerprint f{tag * 0x9E3779B97F4A7C15ull, ~tag};
    for (const auto& [a, b] : parts) {
        f.lo = bits::mix64(f.lo ^ a) + 0x27D4EB2F165667C5ull;
        f.hi = bits::mix64(f.hi + b) ^ 0x94D049BB133111EBull;
    }
    return f;
}
}

Fingerprint Engine::fingerprint(const SymState& s) {
    std::vector<std::pair<std::uint64_t, std::uint64_t>> parts{shash(s.theta), shash(s.delta)};
    for (Ref o : s.om) parts.push_back(shash(o));
    return combine(0x5359'4D00ull + s.hid.size(), parts);
}

Fingerprint Engine::invariant(const SymState& s) {
    if (s.hid.empty()) return fingerprint(s);
    // Isomorphism invariants of the contracted model: the projections onto
    // the valuation, and counts (worlds, designated worlds, worlds per atom,
    // edges per agent). Counts are exact below 2^53, and deterministic above.
    std::vector<std::uint32_t> hxy;
    for (std::uint32_t k : s.hid) { hxy.push_back(hv(k, 0)); hxy.push_back(hv(k, 1)); }
    const Ref HX = vars(s.hid, 0, false), HXY = m_.varset(hxy);
    const std::vector<std::uint32_t> xs = xvars(s.hid);
    std::vector<std::uint32_t> xys = xs;
    for (std::uint32_t v : xs) xys.push_back(v + 1);
    auto num = [](double d) { std::uint64_t u; std::memcpy(&u, &d, sizeof u); return std::pair<std::uint64_t, std::uint64_t>{u, ~u}; };
    std::vector<std::pair<std::uint64_t, std::uint64_t>> parts{
        shash(m_.exists(s.theta, HX)), shash(m_.exists(s.delta, HX)),
        num(m_.count(s.theta, xs)), num(m_.count(s.delta, xs))};
    for (Ref o : s.om) { parts.push_back(shash(m_.exists(o, HXY))); parts.push_back(num(m_.count(o, xys))); }
    for (std::uint32_t p = 0; p < na_; ++p) {
        const Ref t = m_.band(s.theta, m_.var(av(p, 0)));
        if (t != s.theta && t != kFalse) parts.push_back(num(m_.count(t, xs)));
    }
    return combine(0x494E'5600ull, parts);
}

bool Engine::equivalent(const SymState& a, const SymState& b) {
    if (a.hid.empty() && b.hid.empty()) return fingerprint(a) == fingerprint(b);
    ++stats.equivalence_checks;
    if (relabels_onto(a, b)) { ++stats.equivalent_by_relabel; return true; }
    // Disjoint union: both on slots 0..m-1 (padded with zeros), tag slot m.
    const std::uint32_t m = static_cast<std::uint32_t>(std::max(a.hid.size(), b.hid.size()));
    std::vector<std::uint32_t> hid;
    for (std::uint32_t k = 0; k <= m; ++k) hid.push_back(k);
    auto side = [&](const SymState& s, bool tag, unsigned c) {
        std::vector<std::pair<std::uint32_t, bool>> lits;
        for (std::uint32_t k = static_cast<std::uint32_t>(s.hid.size()); k < m; ++k) lits.push_back({hv(k, c), false});
        lits.push_back({hv(m, c), tag});
        return m_.cube(std::move(lits));
    };
    const Ref ax = side(a, false, 0), ay = side(a, false, 1), bx = side(b, true, 0), by = side(b, true, 1);
    const Ref ta = m_.band(a.theta, ax), tb = m_.band(b.theta, bx);
    const Ref theta = m_.bor(ta, tb);
    const Ref delta = m_.bor(m_.band(a.delta, ax), m_.band(b.delta, bx));
    std::vector<Ref> om(ng_);
    for (AgentIdx g = 0; g < ng_; ++g)
        om[g] = m_.bor(m_.band(m_.band(a.om[g], ax), ay), m_.band(m_.band(b.om[g], bx), by));
    // Both sides are contracted: within a side only equal worlds are
    // bisimilar. Across, candidates must agree at depth one; a world left
    // without any candidate decides the question at once.
    const Ref Y = vars(hid, 1);
    const Ref tag_x = m_.var(hv(m, 0)), tag_y = m_.var(hv(m, 1));
    const Ref tt = m_.band(theta, m_.permute(theta, XY_, true));
    const Ref cross = m_.band(m_.band(m_.band(tt, eqP_), m_.biff(delta, m_.permute(delta, XY_, true))),
                              m_.bxor(tag_x, tag_y));
    const Ref D = depth_one_pairs(hid, theta, delta, om, cross);
    const Ref has = m_.exists(D, Y);
    if (!m_.subset(theta, has)) return false;
    const Ref E = bisimulation(hid, theta, delta, om, m_.bor(same_world(hid, theta), D));
    // A total bisimulation between them is an isomorphism of the quotients.
    const Ref a_matched = m_.and_exists(E, m_.permute(tb, XY_, true), Y);
    if (!m_.subset(ta, a_matched)) return false;
    const Ref b_matched = m_.and_exists(E, m_.permute(ta, XY_, true), Y);
    return m_.subset(tb, b_matched);
}

Engine::SwapFilter Engine::swap_filter(const SymState& s) {
    SwapFilter f;
    const std::vector<std::uint32_t> xs = xvars(s.hid);
    std::vector<std::uint32_t> xys = xs;
    for (std::uint32_t v : xs) xys.push_back(v + 1);
    for (Ref o : s.om) f.edges.push_back(m_.count(o, xys));
    for (std::uint32_t p = 0; p < na_; ++p) f.atoms.push_back(m_.count(m_.band(s.theta, m_.var(av(p, 0))), xs));
    return f;
}

// Sufficient test for a ≅ b on one set of hidden slots: worlds of a and b
// that agree at depth one (valuation, designation, and per agent the sets of
// successor valuations) are candidate partners. When the candidates, read on
// the hidden bits alone, form a bijection π of hidden values, renaming a's
// hidden bits by π and obtaining b's four BDDs proves an isomorphism. False
// means "not shown", not "not bisimilar".
bool Engine::relabels_onto(const SymState& a, const SymState& b) {
    if (a.hid != b.hid) return false;
    const Ref YH = vars(a.hid, 1, false), XH = vars(a.hid, 0, false);
    std::vector<std::uint32_t> up, pp;
    for (std::uint32_t p = 0; p < na_; ++p) { up.push_back(av(p, 2)); pp.push_back(av(p, 0)); pp.push_back(av(p, 1)); }
    const Ref UP = m_.varset(up), PXY = m_.varset(pp);
    const Ref ay = m_.permute(a.theta, XY_, true), by = m_.permute(b.theta, XY_, true);
    const Ref ady = m_.permute(a.delta, XY_, true), bdy = m_.permute(b.delta, XY_, true);
    Ref D = m_.band(m_.band(m_.band(a.theta, by), eqP_), m_.biff(a.delta, bdy));
    for (AgentIdx g = 0; g < ng_ && D != kFalse; ++g)
        for (int des = 1; des >= 0 && D != kFalse; --des) {
            const Ref Sa = m_.and_exists(a.om[g], des ? ady : m_.bdiff(ay, ady), YH);
            const Ref Sb = m_.and_exists(b.om[g], des ? bdy : m_.bdiff(by, bdy), YH);
            const Ref diff = m_.bxor(m_.permute(Sa, YU_, true), m_.permute(Sb, XY_YU_, true));
            D = m_.bdiff(D, m_.and_exists(D, diff, UP));
        }
    if (D == kFalse) return false;
    const Ref M = m_.exists(D, PXY);                        // M(x_H, y_H)
    // π must be a bijection: functional and injective.
    Ref neqYU = kFalse, neqXU = kFalse;
    for (std::uint32_t k : a.hid) {
        neqYU = m_.bor(neqYU, m_.bxor(m_.var(hv(k, 1)), m_.var(hv(k, 2))));
        neqXU = m_.bor(neqXU, m_.bxor(m_.var(hv(k, 0)), m_.var(hv(k, 2))));
    }
    const Ref M_xu = m_.permute(M, YU_, true);              // M(x_H, u_H)
    if (m_.band(m_.band(M, M_xu), neqYU) != kFalse) return false;
    const Ref M_uy = m_.permute(M, X_U_, false);            // M(u_H, y_H)
    if (m_.band(m_.band(M, M_uy), neqXU) != kFalse) return false;
    // Rename a by π and compare.
    const auto rel_x = [&](Ref f) {                        // f(x) ↦ f(π⁻¹ x)
        return m_.permute(m_.and_exists(f, M, XH), YX_, true);
    };
    if (rel_x(a.theta) != b.theta || rel_x(a.delta) != b.delta) return false;
    const Ref M_yu = m_.permute(M, XY_YU_, true);           // M(y_H, u_H)
    const Ref UH = vars(a.hid, 2, false);
    for (AgentIdx g = 0; g < ng_; ++g) {
        const Ref o1 = m_.permute(m_.and_exists(a.om[g], M_xu, XH), UX_, false);   // source renamed
        const Ref o2 = m_.permute(m_.and_exists(o1, M_yu, YH), UY_, true);         // target renamed
        (void)UH;
        if (o2 != b.om[g]) return false;
    }
    return true;
}

bool Engine::fixes(const SymState& s, const AgentSymmetry::Swap& sw) {
    std::vector<std::uint32_t> perm(m_.num_vars());
    for (std::uint32_t v = 0; v < perm.size(); ++v) perm[v] = v;
    for (std::uint32_t p = 0; p < na_; ++p)
        for (unsigned c = 0; c < 2; ++c) perm[av(p, c)] = av(sw.atom[p], c);
    auto swapped = [&](AgentIdx g) { return g == sw.a ? sw.b : g == sw.b ? sw.a : g; };
    if (s.hid.empty()) {
        // Canonical encoding: the swap fixes s iff it fixes every BDD.
        if (m_.permute(s.theta, perm, false) != s.theta) return false;
        if (m_.permute(s.delta, perm, false) != s.delta) return false;
        for (AgentIdx g = 0; g < ng_; ++g) {
            const AgentIdx h = swapped(g);
            if (h < g) continue;                       // the pair was checked from h
            if (m_.permute(s.om[g], perm, false) != s.om[h]) return false;
            if (h != g && m_.permute(s.om[h], perm, false) != s.om[g]) return false;
        }
        return true;
    }
    // Hidden slots stay where they are; the renamed state is compared with s
    // by its invariant, then exactly.
    SymState r;
    r.hid = s.hid;
    r.theta = m_.permute(s.theta, perm, false);
    r.delta = m_.permute(s.delta, perm, false);
    r.om.resize(ng_);
    for (AgentIdx g = 0; g < ng_; ++g) r.om[swapped(g)] = m_.permute(s.om[g], perm, false);
    if (r.theta == s.theta && r.delta == s.delta && r.om == s.om) { ++stats.fixes_same; return true; }
    if (relabels_onto(r, s)) { ++stats.fixes_relabel; return true; }
    if (invariant(r) != invariant(s)) { ++stats.fixes_invariant; return false; }
    const bool eq = equivalent(r, s);
    ++(eq ? stats.fixes_equivalent : stats.fixes_rejected);
    return eq;
}

// ---------------------------------------------------------------- testing

EpistemicState Engine::explicit_state(const SymState& s) {
    std::vector<std::uint32_t> xv, yv;
    for (std::uint32_t p = 0; p < na_; ++p) xv.push_back(av(p, 0));
    for (std::uint32_t k : s.hid) xv.push_back(hv(k, 0));
    std::sort(xv.begin(), xv.end());
    for (std::uint32_t v : xv) yv.push_back(v + 1);
    // Enumeration visits assignments in lexicographic order of xv, so world
    // indices are found by binary search.
    std::vector<std::vector<std::uint8_t>> ws;
    m_.enumerate(s.theta, xv, ws);
    auto index = [&](const std::vector<std::uint8_t>& v) {
        const auto it = std::lower_bound(ws.begin(), ws.end(), v);
        if (it == ws.end() || *it != v) throw std::logic_error("explicit_state: successor outside theta");
        return static_cast<WorldIdx>(it - ws.begin());
    };
    std::vector<std::size_t> atom_pos(na_);
    for (std::uint32_t p = 0; p < na_; ++p)
        atom_pos[p] = std::lower_bound(xv.begin(), xv.end(), av(p, 0)) - xv.begin();

    EpistemicState e;
    e.allocate(static_cast<std::uint32_t>(ws.size()), na_, ng_);
    std::vector<Ref> cubes(ws.size());
    for (std::size_t i = 0; i < ws.size(); ++i) {
        std::vector<std::pair<std::uint32_t, bool>> lits;
        for (std::size_t j = 0; j < xv.size(); ++j) lits.push_back({xv[j], bool(ws[i][j])});
        cubes[i] = m_.cube(std::move(lits));
        for (std::uint32_t p = 0; p < na_; ++p)
            if (ws[i][atom_pos[p]]) e.set_atom(static_cast<WorldIdx>(i), p);
    }
    {
        std::vector<std::vector<std::uint8_t>> ds;
        m_.enumerate(s.delta, xv, ds);
        for (const auto& d : ds) e.set_designated(index(d));
    }
    // Successor sets are BDDs over y, so equal sets are equal nodes: each
    // distinct set is enumerated once.
    std::unordered_map<Ref, std::uint32_t> set_id;
    for (AgentIdx g = 0; g < ng_; ++g)
        for (std::size_t i = 0; i < ws.size(); ++i) {
            const Ref r = m_.restrict_cube(s.om[g], cubes[i]);
            if (r == kFalse) continue;
            auto [it, fresh] = set_id.try_emplace(r, 0);
            if (fresh) {
                std::vector<std::vector<std::uint8_t>> succ;
                m_.enumerate(r, yv, succ);
                std::vector<WorldIdx> list;
                list.reserve(succ.size());
                for (auto& v : succ) list.push_back(index(v));
                std::sort(list.begin(), list.end());
                it->second = e.add_set(list);
            }
            e.set_of[std::size_t(g) * ws.size() + i] = it->second;
        }
    e.invalidate();
    return e;
}

} // namespace sym
