#include "symdel.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

namespace sym {

using bdd::kFalse;
using bdd::kTrue;

namespace {
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
      m_(3 * (static_cast<std::uint32_t>(t.num_atoms()) + kMaxHidden)) {
    XY_    = copy_map({{0, 1}});
    YX_    = copy_map({{1, 0}});
    YU_    = copy_map({{1, 2}});
    SWAP_  = copy_map({{0, 1}, {1, 0}});
    XY_UX_ = copy_map({{0, 2}, {1, 0}});
    X_U_   = copy_map({{0, 2}});
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
    // w ∈ theta with no R_ag-successor outside ext
    const Ref out = m_.neg(m_.permute(ext, XY_, true));
    const Ref bad = m_.and_exists(s.om[ag], out, vars(s.hid, 1));
    return m_.bdiff(s.theta, bad);
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
    std::vector<Ref> cx(nw), cy(nw);
    for (WorldIdx w = 0; w < nw; ++w) { cx[w] = world_cube(w, 0); cy[w] = world_cube(w, 1); }
    for (WorldIdx w = 0; w < nw; ++w) {
        s.theta = m_.bor(s.theta, cx[w]);
        if (e.is_designated(w)) s.delta = m_.bor(s.delta, cx[w]);
    }
    std::vector<Ref> setb(e.num_sets(), kFalse);
    for (std::uint32_t id = 0; id < e.num_sets(); ++id)
        for (WorldIdx v : e.set(id)) setb[id] = m_.bor(setb[id], cy[v]);
    for (AgentIdx g = 0; g < ng_; ++g) {
        std::vector<Ref> src(e.num_sets(), kFalse);
        for (WorldIdx w = 0; w < nw; ++w) src[e.succ_set(g, w)] = m_.bor(src[e.succ_set(g, w)], cx[w]);
        Ref o = kFalse;
        for (std::uint32_t id = 0; id < e.num_sets(); ++id)
            if (src[id] != kFalse && setb[id] != kFalse) o = m_.bor(o, m_.band(src[id], setb[id]));
        s.om.push_back(o);
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

SymState Engine::contract(const SymState& s) {
    const Ref X = vars(s.hid, 0), Y = vars(s.hid, 1), U = vars(s.hid, 2);
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
    const Ref theta = reach, theta_y = m_.permute(theta, XY_, true), tt = m_.band(theta, theta_y);
    std::vector<Ref> om(ng_);
    for (AgentIdx g = 0; g < ng_; ++g) om[g] = m_.band(s.om[g], tt);
    const Ref delta = m_.band(s.delta, theta);

    // 2. maximal bisimulation E(x,y), a greatest fixpoint
    Ref E = m_.band(m_.band(tt, eqP_), m_.biff(delta, m_.permute(delta, XY_, true)));
    std::vector<Ref> om_sw(ng_), om_xu(ng_);
    for (AgentIdx g = 0; g < ng_; ++g) {
        om_sw[g] = m_.permute(om[g], SWAP_, false);   // R(y -> x)
        om_xu[g] = m_.permute(om[g], YU_, true);      // R(x -> u)
    }
    while (true) {
        const Ref E_ux = m_.permute(E, XY_UX_, false);
        Ref E2 = E;
        for (AgentIdx g = 0; g < ng_ && E2 != kFalse; ++g) {
            const Ref A = m_.and_exists(om_sw[g], E_ux, X);               // A(y,u)
            const Ref bad = m_.and_exists(om_xu[g], m_.neg(A), U);        // x has an unmatched succ
            const Ref F = m_.neg(bad);
            E2 = m_.band(E2, m_.band(F, m_.permute(F, SWAP_, false)));
        }
        if (E2 == E) break;
        E = E2;
    }

    // 3. one representative per class: the least hidden assignment
    Ref rep = theta;
    if (!s.hid.empty()) {
        Ref lt = kFalse, eq = kTrue;                 // y_H <lex x_H
        for (std::uint32_t k : s.hid) {
            const Ref hx = m_.var(hv(k, 0)), hy = m_.var(hv(k, 1));
            lt = m_.bor(lt, m_.band(eq, m_.band(m_.neg(hy), hx)));
            eq = m_.band(eq, m_.biff(hx, hy));
        }
        rep = m_.bdiff(theta, m_.and_exists(E, lt, Y));
    }
    const Ref rep_y = m_.permute(rep, XY_, true), rr = m_.band(rep, rep_y);
    const Ref E_uy = m_.permute(E, X_U_, false);
    SymState q;
    q.hid = s.hid;
    q.theta = rep;
    q.delta = m_.band(delta, rep);
    for (AgentIdx g = 0; g < ng_; ++g) q.om.push_back(m_.band(rr, m_.and_exists(om_xu[g], E_uy, U)));

    // 4. drop hidden slots determined by the other variables on theta
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

    // 4b. re-encode the N distinct hidden vectors onto ceil(log2 N) fresh slots,
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

    // 5. normalise: remaining slots move to 0..m-1 (order-preserving)
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

// ---------------------------------------------------------------- testing

EpistemicState Engine::explicit_state(const SymState& s) {
    std::vector<std::uint32_t> xv, yv;
    for (std::uint32_t p = 0; p < na_; ++p) xv.push_back(av(p, 0));
    for (std::uint32_t k : s.hid) xv.push_back(hv(k, 0));
    std::sort(xv.begin(), xv.end());
    for (std::uint32_t v : xv) yv.push_back(v + 1);
    std::vector<std::vector<std::uint8_t>> ws;
    m_.enumerate(s.theta, xv, ws);
    std::map<std::vector<std::uint8_t>, WorldIdx> idx;
    for (std::size_t i = 0; i < ws.size(); ++i) idx[ws[i]] = static_cast<WorldIdx>(i);

    EpistemicState e;
    e.allocate(static_cast<std::uint32_t>(ws.size()), na_, ng_);
    std::vector<Ref> cubes(ws.size());
    for (std::size_t i = 0; i < ws.size(); ++i) {
        std::vector<std::pair<std::uint32_t, bool>> lits;
        for (std::size_t j = 0; j < xv.size(); ++j) lits.push_back({xv[j], bool(ws[i][j])});
        cubes[i] = m_.cube(std::move(lits));
        for (std::uint32_t p = 0; p < na_; ++p) {
            const auto pos = std::lower_bound(xv.begin(), xv.end(), av(p, 0)) - xv.begin();
            if (ws[i][pos]) e.set_atom(static_cast<WorldIdx>(i), p);
        }
        if (m_.restrict_cube(s.delta, cubes[i]) == kTrue) e.set_designated(static_cast<WorldIdx>(i));
    }
    for (AgentIdx g = 0; g < ng_; ++g)
        for (std::size_t i = 0; i < ws.size(); ++i) {
            std::vector<std::vector<std::uint8_t>> succ;
            m_.enumerate(m_.restrict_cube(s.om[g], cubes[i]), yv, succ);
            std::vector<WorldIdx> list;
            for (auto& v : succ) list.push_back(idx.at(v));
            std::sort(list.begin(), list.end());
            if (!list.empty()) e.set_of[std::size_t(g) * ws.size() + i] = e.add_set(list);
        }
    e.invalidate();
    return e;
}

} // namespace sym
