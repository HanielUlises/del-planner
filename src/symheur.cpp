#include "symheur.hpp"

namespace sym {

using bdd::kFalse;
using bdd::kTrue;

std::vector<std::uint8_t> initial_facts(Engine& eng, SymState& s,
                                        const KnowledgeRelaxationHeuristic& h) {
    const auto& facts = h.fact_formulas();
    std::vector<std::uint8_t> init(facts.size(), 0);
    for (std::size_t f = 0; f < facts.size(); ++f)
        if (facts[f]) init[f] = eng.mgr().band(s.delta, eng.sat(s, *facts[f])) != kFalse;
    return init;
}

Relaxation::Relaxation(Engine& eng, const PlanningTask& t, SymState& s)
    : e_(eng), m_(eng.mgr()), t_(t), s_(s) {
    theta_ = s.theta;
    delta_ = s.delta;
    theta_y_ = to_y(theta_);
    ycube_ = e_.vars(s.hid, 1);
    const std::size_t P = t.num_atoms(), A = t.actions.size(), G = t.num_agents();
    canT_.resize(P);
    canF_.resize(P);
    for (AtomIdx p = 0; p < P; ++p) {
        const Ref v = m_.var(e_.av(p, 0));
        canT_[p] = m_.band(theta_, v);
        canF_[p] = m_.bdiff(theta_, v);
    }
    std::uint32_t ne = 0;
    ev_base_.resize(A);
    for (std::size_t a = 0; a < A; ++a) { ev_base_[a] = ne; ne += static_cast<std::uint32_t>(t.actions[a].events.size()); }
    genT_.assign(ne, kFalse);
    genF_.assign(ne, kFalse);
    app_.assign(A, 0);
    type_base_.resize(A * G);
    for (std::size_t a = 0; a < A; ++a)
        for (AgentIdx i = 0; i < G; ++i) {
            type_base_[a * G + i] = static_cast<std::uint32_t>(type_.size());
            type_.resize(type_.size() + 1 + (i < t.actions[a].obs_cases.size() ? t.actions[a].obs_cases[i].size() : 0), 0);
        }
}

Ref Relaxation::to_y(Ref f) { return m_.permute(f, e_.XY_, true); }

const ObsCase& Relaxation::obs(const Action& a, AgentIdx i, std::size_t t) const {
    return t == 0 ? a.default_obs : a.obs_cases[i][t - 1];
}

void Relaxation::build_cuts() {
    const std::size_t G = t_.num_agents();
    guar_.assign(G, kFalse);
    for (AgentIdx i = 0; i < G; ++i) {
        Ref cut = kFalse;
        for (std::size_t a = 0; a < t_.actions.size(); ++a) {
            if (!app_[a]) continue;
            const Action& A = t_.actions[a];
            const std::uint32_t b = ev_base_[a], tb = type_base_[a * G + i];
            const std::size_t nt = 1 + (i < A.obs_cases.size() ? A.obs_cases[i].size() : 0);
            for (EventIdx e = 0; e < A.events.size(); ++e) {
                if (genT_[b + e] == kFalse) continue;
                Ref sep = kFalse;              // ⋁_t ⋀_{f ∈ Q_t(e)} genF_f(y)
                for (std::size_t ty = 0; ty < nt; ++ty) {
                    if (!type_[tb + ty]) continue;
                    Ref inter = theta_y_;
                    const auto row = obs(A, i, ty).event_row(e);
                    for (EventIdx f = 0; f < A.events.size() && inter != kFalse; ++f)
                        if (bits::test(row, f)) inter = m_.band(inter, to_y(genF_[b + f]));
                    sep = m_.bor(sep, inter);
                }
                if (sep != kFalse) cut = m_.bor(cut, m_.band(genT_[b + e], sep));
            }
        }
        guar_[i] = m_.bdiff(s_.om[i], cut);
    }
    cuts_ready_ = true;
}

Relaxation::TV Relaxation::box(AgentIdx i, TV psi) {
    if (!cuts_ready_) build_cuts();
    const Ref base = m_.bor(elim_, psi.T);
    const Ref bad = m_.and_exists(guar_[i], m_.neg(to_y(base)), ycube_);
    return {m_.bdiff(theta_, bad), m_.band(theta_, m_.and_exists(s_.om[i], to_y(psi.F), ycube_))};
}

Relaxation::TV Relaxation::eval(const Formula& f) {
    if (auto it = memo_.find(f.id); it != memo_.end()) return it->second;
    TV r{kFalse, kFalse};
    switch (f.kind) {
    case FormulaKind::Top:  r = {theta_, kFalse}; break;
    case FormulaKind::Bot:  r = {kFalse, theta_}; break;
    case FormulaKind::Atom: r = {canT_[f.atom], canF_[f.atom]}; break;
    case FormulaKind::Not: {
        const TV c = eval(*f.children[0]);
        r = {c.F, c.T};
        break;
    }
    case FormulaKind::And:
    case FormulaKind::Or: {
        const bool conj = f.kind == FormulaKind::And;
        r = conj ? TV{theta_, kFalse} : TV{kFalse, theta_};
        for (const auto& c : f.children) {
            const TV k = eval(*c);
            if (conj) { r.T = m_.band(r.T, k.T); r.F = m_.bor(r.F, k.F); }
            else      { r.T = m_.bor(r.T, k.T);  r.F = m_.band(r.F, k.F); }
        }
        break;
    }
    case FormulaKind::Belief: r = box(f.agent, eval(*f.children[0])); break;
    case FormulaKind::Kw: {
        const TV c = eval(*f.children[0]);
        const TV p = box(f.agent, c), n = box(f.agent, {c.F, c.T});
        r = {m_.bor(p.T, n.T), m_.band(p.F, n.F)};
        break;
    }
    case FormulaKind::Common: {
        if (!cuts_ready_) build_cuts();
        const TV c = eval(*f.children[0]);
        Ref orig = kFalse, guar = kFalse;
        for (AgentIdx g : f.group) { orig = m_.bor(orig, s_.om[g]); guar = m_.bor(guar, guar_[g]); }
        guar = m_.band(guar, m_.neg(to_y(elim_)));
        Ref Y = kFalse;                               // F: least fixpoint
        while (true) {
            const Ref Z = m_.band(theta_, m_.and_exists(orig, to_y(m_.bor(c.F, Y)), ycube_));
            if (Z == Y) break;
            Y = Z;
        }
        Ref X = theta_;                               // T: greatest fixpoint
        while (true) {
            const Ref Z = m_.bdiff(theta_, m_.and_exists(guar, m_.neg(to_y(m_.band(c.T, X))), ycube_));
            if (Z == X) break;
            X = Z;
        }
        r = {X, Y};
        break;
    }
    }
    memo_.emplace(f.id, r);
    return r;
}

bool Relaxation::reached(const Formula& f) {
    const TV g = eval(f);
    if (m_.band(delta_, g.T) == kFalse) return false;
    return m_.subset(delta_, m_.bor(g.T, elimD_));
}

bool Relaxation::step() {
    if (std::chrono::steady_clock::now() > until) { timed_out = true; return false; }
    std::vector<Ref> nT = canT_, nF = canF_, ngT = genT_, ngF = genF_;
    Ref nelim = elim_, nelimD = elimD_;
    std::vector<std::uint8_t> napp = app_, ntype = type_;
    const std::size_t G = t_.num_agents();
    const Ref sure_des = m_.bdiff(delta_, elimD_);
    for (std::size_t a = 0; a < t_.actions.size(); ++a) {
        const Action& A = t_.actions[a];
        const std::uint32_t b = ev_base_[a];
        for (EventIdx e = 0; e < A.events.size(); ++e) {
            const TV p = eval(*A.events[e].precondition);
            ngT[b + e] = m_.bor(ngT[b + e], p.T);
            ngF[b + e] = m_.bor(ngF[b + e], p.F);
        }
        bool app = napp[a];
        for (EventIdx e : A.designated_events)
            if (!app && m_.band(delta_, ngT[b + e]) != kFalse) app = true;
        napp[a] = app;
        if (!app) continue;
        for (AgentIdx i = 0; i < G; ++i) {
            const std::uint32_t tb = type_base_[a * G + i];
            ntype[tb] = 1;
            for (std::size_t c = 0; i < A.obs_cases.size() && c < A.obs_cases[i].size(); ++c)
                if (m_.band(delta_, eval(*A.obs_cases[i][c].condition).T) != kFalse) ntype[tb + 1 + c] = 1;
        }
        Ref allfail = theta_;
        for (EventIdx e = 0; e < A.events.size(); ++e) {
            const Event& ev = A.events[e];
            allfail = m_.band(allfail, ngF[b + e]);
            for (const auto& [p, phi] : ev.post_true)
                nT[p] = m_.bor(nT[p], m_.band(eval(*phi).T, ngT[b + e]));
            for (const auto& [p, phi] : ev.post_false)
                nF[p] = m_.bor(nF[p], m_.band(eval(*phi).T, ngT[b + e]));
        }
        // Applicability gives every designated world a designated descendant.
        allfail = m_.bdiff(allfail, sure_des);
        nelim = m_.bor(nelim, allfail);
        if (A.designated_events.size() > 1)
            for (EventIdx e : A.designated_events) nelimD = m_.bor(nelimD, m_.band(ngF[b + e], delta_));
    }
    nelimD = m_.bor(nelimD, m_.band(nelim, delta_));

    const bool changed = nT != canT_ || nF != canF_ || nelim != elim_ || nelimD != elimD_ ||
                         ngT != genT_ || ngF != genF_ || napp != app_ || ntype != type_;
    canT_.swap(nT); canF_.swap(nF); genT_.swap(ngT); genF_.swap(ngF);
    elim_ = nelim; elimD_ = nelimD;
    app_.swap(napp); type_.swap(ntype);
    ++layer_;
    memo_.clear();
    cuts_ready_ = false;
    return changed;
}

std::uint32_t h_delta(Engine& eng, const PlanningTask& t, SymState& s) {
    Relaxation r(eng, t, s);
    for (;;) {
        if (r.goal_reached()) return r.layer();
        if (r.layer() >= r.max_layers || !r.step()) return Relaxation::kInf;
    }
}

std::optional<std::uint32_t> h_delta_until(Engine& eng, const PlanningTask& t, SymState& s,
                                           std::chrono::steady_clock::time_point until) {
    Relaxation r(eng, t, s);
    r.until = until;
    for (;;) {
        if (r.goal_reached()) return r.layer();
        if (r.layer() >= r.max_layers) return Relaxation::kInf;
        if (!r.step()) return r.timed_out ? std::nullopt : std::optional<std::uint32_t>(Relaxation::kInf);
    }
}

Estimates h_delta_both(Engine& eng, const PlanningTask& t, SymState& s) {
    Relaxation r(eng, t, s);
    std::vector<const Formula*> cs;
    if (t.goal->kind == FormulaKind::And) for (const auto& c : t.goal->children) cs.push_back(c.get());
    else cs.push_back(t.goal.get());
    std::vector<std::uint32_t> first(cs.size(), Relaxation::kInf);
    Estimates out;
    std::size_t open = cs.size();
    for (;;) {
        if (out.h == Relaxation::kInf && r.goal_reached()) out.h = r.layer();
        for (std::size_t k = 0; k < cs.size(); ++k)
            if (first[k] == Relaxation::kInf && r.reached(*cs[k])) { first[k] = r.layer(); --open; }
        if ((out.h != Relaxation::kInf && open == 0) || r.layer() >= r.max_layers || !r.step()) break;
    }
    if (open == 0) { out.sum = 0; for (auto l : first) out.sum += l; }
    return out;
}

} // namespace sym
