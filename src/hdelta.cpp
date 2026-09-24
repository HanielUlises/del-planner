#include "hdelta.hpp"

#include <cmath>

namespace hdelta {
namespace {

using bits::Word;

void copy_row(Word* d, const Word* s, std::uint32_t rw) { for (std::uint32_t k = 0; k < rw; ++k) d[k] = s[k]; }
void or_row(Word* d, const Word* s, std::uint32_t rw)   { for (std::uint32_t k = 0; k < rw; ++k) d[k] |= s[k]; }
void and_row(Word* d, const Word* s, std::uint32_t rw)  { for (std::uint32_t k = 0; k < rw; ++k) d[k] &= s[k]; }
bool test_row(const Word* r, std::uint32_t i) { return (r[i / 64] >> (i % 64)) & 1u; }
void set_row(Word* r, std::uint32_t i) { r[i / 64] |= Word{1} << (i % 64); }

// φ is a Boolean combination of formulas [i]ψ and Kw_i ψ (and constants).
bool introspective(const Formula& f, AgentIdx i) {
    switch (f.kind) {
    case FormulaKind::Top: case FormulaKind::Bot: return true;
    case FormulaKind::Belief: case FormulaKind::Kw: return f.agent == i;
    case FormulaKind::Not: case FormulaKind::And: case FormulaKind::Or:
        for (const auto& c : f.children) if (!introspective(*c, i)) return false;
        return true;
    default: return false;
    }
}

bool row_reflexive(const ObsCase& c, std::size_t ne) {
    for (EventIdx e = 0; e < ne; ++e) if (!bits::test(c.event_row(e), e)) return false;
    return true;
}

// K45: f ∈ Q(e) implies Q(f) = Q(e).
bool row_k45(const ObsCase& c, std::size_t ne) {
    for (EventIdx e = 0; e < ne; ++e) {
        bool ok = true;
        bits::for_each_until(c.event_row(e), [&](std::uint32_t f) {
            ok = bits::equal(c.event_row(e), c.event_row(f));
            return ok;
        });
        if (!ok) return false;
    }
    return true;
}

} // namespace

Relaxation::Relaxation(const PlanningTask& t, const EpistemicState& s)
    : t_(t), s_(s), n_(s.num_worlds), rw_(static_cast<std::uint32_t>(bits::words_for(s.num_worlds))) {
    all_.assign(rw_, ~Word{0});
    if (rw_) all_.back() &= bits::tail_mask(n_);
    desig_.assign(s.designated_bits().begin(), s.designated_bits().end());
    const std::size_t P = t.num_atoms(), A = t.actions.size(), G = t.num_agents();
    canT_.assign(P * rw_, 0);
    canF_.assign(P * rw_, 0);
    for (WorldIdx w = 0; w < n_; ++w)
        for (AtomIdx p = 0; p < P; ++p)
            set_row(row(s.has_atom(w, p) ? canT_ : canF_, p), w);
    elim_.assign(rw_, 0);
    elimD_.assign(rw_, 0);
    ev_base_.resize(A);
    for (std::size_t a = 0; a < A; ++a) {
        ev_base_[a] = num_ev_;
        num_ev_ += t.actions[a].events.size();
        ev_action_.resize(num_ev_, static_cast<std::uint32_t>(a));
    }
    genT_.assign(std::size_t(num_ev_) * rw_, 0);
    genF_.assign(std::size_t(num_ev_) * rw_, 0);
    app_.assign(A, 0);
    type_base_.resize(A * G);
    for (std::size_t a = 0; a < A; ++a)
        for (AgentIdx i = 0; i < G; ++i) {
            type_base_[a * G + i] = static_cast<std::uint32_t>(type_.size());
            type_.resize(type_.size() + 1 + (i < t.actions[a].obs_cases.size() ? t.actions[a].obs_cases[i].size() : 0), 0);
        }
    const std::size_t u = formula_universe_size();
    slot_.assign(u, 0);
    stamp_.assign(u, 0);
    arena_.reserve((u + 8) * 2 * std::size_t(rw_));
    sig_words_ = static_cast<std::uint32_t>(bits::words_for(num_ev_));

    // Frame facts. Seriality repair deletes worlds the relaxation does not model,
    // so both refinements are off under it (and h^Δ itself assumes it is off).
    refl_.assign(G, 0);
    k45_.assign(G, 0);
    for (AgentIdx i = 0; i < G && !t.repair_seriality(); ++i) {
        bool refl = true, k45 = true;
        for (WorldIdx w = 0; w < n_; ++w) {
            const auto S = s.succ(i, w);
            if (!std::binary_search(S.begin(), S.end(), w)) refl = false;
            // Contracted states intern sets by content, so equal ids decide
            // most pairs; contents are compared only when the ids differ.
            const std::uint32_t sid = s.succ_set(i, w);
            for (WorldIdx v : S) {
                if (s.succ_set(i, v) == sid) continue;
                const auto Sv = s.succ(i, v);
                if (!std::equal(S.begin(), S.end(), Sv.begin(), Sv.end())) { k45 = false; break; }
            }
        }
        for (const Action& act : t.actions) {
            const std::size_t ne = act.events.size();
            auto check = [&](const ObsCase& c) { refl = refl && row_reflexive(c, ne); k45 = k45 && row_k45(c, ne); };
            check(act.default_obs);
            if (i < act.obs_cases.size()) for (const ObsCase& c : act.obs_cases[i]) check(c);
        }
        refl_[i] = refl;
        k45_[i] = k45;
    }
    intro_.assign(std::size_t(num_ev_) * G, 0);
    for (std::size_t a = 0; a < A; ++a)
        for (std::size_t e = 0; e < t.actions[a].events.size(); ++e)
            for (AgentIdx i = 0; i < G; ++i)
                intro_[std::size_t(ev_base_[a] + e) * G + i] =
                    k45_[i] && introspective(*t.actions[a].events[e].precondition, i);
    public_ = !t.repair_seriality();
    for (const Action& act : t.actions) {
        auto identity = [&](const ObsCase& c) {
            for (EventIdx e = 0; e < act.events.size(); ++e)
                if (bits::count(c.event_row(e)) != 1 || !bits::test(c.event_row(e), e)) return false;
            return true;
        };
        for (AgentIdx i = 0; i < G && public_; ++i) {
            public_ = identity(act.default_obs);
            if (i < act.obs_cases.size()) for (const ObsCase& c : act.obs_cases[i]) public_ = public_ && identity(c);
        }
    }
    intro_any_.assign(num_ev_, 0);
    for (std::uint32_t g = 0; g < num_ev_; ++g)
        for (AgentIdx i = 0; i < G; ++i) intro_any_[g] |= intro_[std::size_t(g) * G + i];
    xrow_.assign(rw_, 0);
    mask_.assign(rw_, 0);
    scratch_.assign(5 * std::size_t(rw_), 0);
}

std::size_t Relaxation::alloc() {
    const std::size_t off = arena_.size();
    arena_.resize(off + 2 * std::size_t(rw_), 0);
    return off;
}

const ObsCase& Relaxation::obs(const Action& a, AgentIdx i, std::size_t t) const {
    return t == 0 ? a.default_obs : a.obs_cases[i][t - 1];
}

void Relaxation::build_sigs() {
    BS per(std::size_t(n_) * sig_words_, 0);
    for (std::uint32_t a = 0; a < app_.size(); ++a) {
        if (!app_[a]) continue;
        for (std::uint32_t e = 0; e < t_.actions[a].events.size(); ++e) {
            const std::uint32_t g = ev_base_[a] + e;
            bits::for_each(Span(row(genT_, g), rw_), [&](std::uint32_t x) {
                per[std::size_t(x) * sig_words_ + g / 64] |= Word{1} << (g % 64);
            });
        }
    }
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> ids;
    sig_of_.assign(n_, 0);
    sigbits_.clear();
    std::uint32_t count = 0;
    for (WorldIdx x = 0; x < n_; ++x) {
        const Word* r = per.data() + std::size_t(x) * sig_words_;
        std::uint64_t h = 0x9E3779B97F4A7C15ull;
        for (std::uint32_t k = 0; k < sig_words_; ++k) h = bits::mix64(h ^ r[k]);
        auto& cand = ids[h];
        std::uint32_t id = ~0u;
        for (std::uint32_t c : cand)
            if (std::equal(r, r + sig_words_, sigbits_.data() + std::size_t(c) * sig_words_)) { id = c; break; }
        if (id == ~0u) {
            id = count++;
            cand.push_back(id);
            sigbits_.insert(sigbits_.end(), r, r + sig_words_);
        }
        sig_of_[x] = id;
    }
    urow_.clear();
    ucache_.clear();
    brow_.clear();
    bcache_.clear();
    xrow_at_.clear();
    xcache_.clear();
    igroups_.clear();
    gcache_.clear();
    sigs_ready_ = true;
}

// Some active (a, e) at the source and possible type t of agent i with every
// f ∈ Q_t(e) possibly failing at y: the observation possibly separates y.
// ∪_t ∩_{f ∈ Q_t(e)} F(pre f) for global event g and agent i, over i's possible
// types, excluding types exempted by R2. Built once per layer.
const bits::Word* Relaxation::brow(std::uint32_t g, AgentIdx i) {
    const std::size_t G = t_.num_agents();
    if (brow_.size() != std::size_t(num_ev_) * G) brow_.assign(std::size_t(num_ev_) * G, 0);
    std::uint32_t& br = brow_[std::size_t(g) * G + i];
    if (br == 0) {
        const std::uint32_t a = ev_action_[g], e = g - ev_base_[a];
        const Action& A = t_.actions[a];
        const std::uint32_t tb = type_base_[a * G + i];
        const std::size_t nt = 1 + (i < A.obs_cases.size() ? A.obs_cases[i].size() : 0);
        const std::size_t boff = bcache_.size();
        bcache_.resize(boff + 2 * rw_, 0);           // row, then scratch
        Word* B = bcache_.data() + boff;
        Word* tmp = B + rw_;
        const bool intro = intro_[std::size_t(g) * G + i];
        for (std::size_t t = 0; t < nt; ++t) {
            if (!type_[tb + t]) continue;
            // R2: i sees e as e and pre(e) has one value on each i-class.
            if (intro && bits::test(obs(A, i, t).event_row(e), e)) continue;
            copy_row(tmp, all_.data(), rw_);
            bits::for_each(obs(A, i, t).event_row(e), [&](std::uint32_t f) {
                and_row(tmp, row(genF_, ev_base_[a] + f), rw_);
            });
            or_row(B, tmp, rw_);
        }
        br = static_cast<std::uint32_t>(boff / rw_ / 2) + 1;
    }
    return bcache_.data() + std::size_t(br - 1) * 2 * rw_;
}

// Worlds y such that an (x,y) edge of agent i is possibly cut, for x with
// signature sig: the union of brow over the sig's events. Under R4 the events
// with an introspective precondition are left to cut_row_x.
const bits::Word* Relaxation::cut_row(AgentIdx i, std::uint32_t sig) {
    const std::uint64_t key = (std::uint64_t(i) << 32) | sig;
    if (auto it = urow_.find(key); it != urow_.end()) return ucache_.data() + it->second;
    const std::size_t off = ucache_.size();
    ucache_.resize(off + rw_, 0);
    const Span sb(sigbits_.data() + std::size_t(sig) * sig_words_, sig_words_);
    bits::for_each(sb, [&](std::uint32_t g) {
        if (public_ && intro_any_[g]) return;
        const Word* B = brow(g, i);
        or_row(ucache_.data() + off, B, rw_);
    });
    urow_.emplace(key, off);
    return ucache_.data() + off;
}

// cut_row plus, under R4, the introspective events restricted to worlds y that
// no agent they are introspective for relates to x.
const bits::Word* Relaxation::set_row_bits(std::uint32_t sid) {
    if (setrow_at_.size() != s_.num_sets()) setrow_at_.assign(s_.num_sets(), 0);
    if (setrow_at_[sid] == 0) {
        const std::size_t off = setrows_.size();
        setrows_.resize(off + rw_, 0);
        for (WorldIdx y : s_.set(sid)) set_row(setrows_.data() + off, y);
        setrow_at_[sid] = static_cast<std::uint32_t>(off / rw_) + 1;
    }
    return setrows_.data() + std::size_t(setrow_at_[sid] - 1) * rw_;
}

const std::vector<Relaxation::IntroGroup>& Relaxation::intro_groups(AgentIdx i, std::uint32_t sig) {
    const std::uint64_t key = (std::uint64_t(i) << 32) | sig;
    if (auto it = igroups_.find(key); it != igroups_.end()) return it->second;
    std::vector<IntroGroup> groups;
    const Span sb(sigbits_.data() + std::size_t(sig) * sig_words_, sig_words_);
    const std::size_t G = t_.num_agents();
    bits::for_each(sb, [&](std::uint32_t g) {
        if (!intro_any_[g]) return;
        std::uint64_t agents = 0;
        for (AgentIdx k = 0; k < G && k < 64; ++k)
            if (intro_[std::size_t(g) * G + k]) agents |= std::uint64_t{1} << k;
        const Word* B = brow(g, i);
        bool empty = true;
        for (std::uint32_t w = 0; w < rw_ && empty; ++w) empty = B[w] == 0;
        if (empty) return;
        auto it = std::find_if(groups.begin(), groups.end(), [&](const IntroGroup& gr) { return gr.agents == agents; });
        if (it == groups.end()) {
            groups.push_back({agents, gcache_.size()});
            gcache_.resize(gcache_.size() + rw_, 0);
            it = groups.end() - 1;
        }
        or_row(gcache_.data() + it->row, brow(g, i), rw_);
    });
    return igroups_.emplace(key, std::move(groups)).first->second;
}

const bits::Word* Relaxation::cut_row_x(AgentIdx i, WorldIdx x) {
    const Word* U = cut_row(i, sig_of_[x]);
    if (!public_) return U;
    const std::uint64_t key = (std::uint64_t(i) << 32) | x;
    if (auto it = xrow_at_.find(key); it != xrow_at_.end()) return xcache_.data() + it->second;
    const auto& groups = intro_groups(i, sig_of_[x]);
    if (groups.empty()) return U;
    copy_row(xrow_.data(), U, rw_);
    const std::size_t G = t_.num_agents();
    for (const IntroGroup& gr : groups) {
        std::fill(mask_.begin(), mask_.end(), 0);
        for (AgentIdx k = 0; k < G && k < 64; ++k)
            if (gr.agents >> k & 1) or_row(mask_.data(), set_row_bits(s_.succ_set(k, x)), rw_);
        const Word* B = gcache_.data() + gr.row;
        for (std::uint32_t w = 0; w < rw_; ++w) xrow_[w] |= B[w] & ~mask_[w];
    }
    const std::size_t off = xcache_.size();
    xcache_.insert(xcache_.end(), xrow_.begin(), xrow_.end());
    xrow_at_.emplace(key, off);
    return xcache_.data() + off;
}

bool Relaxation::covered(AgentIdx i, WorldIdx x, WorldIdx y) {
    if (!sigs_ready_) build_sigs();
    return test_row(cut_row_x(i, x), y);
}

bool Relaxation::possibly_cut(AgentIdx i, WorldIdx x, WorldIdx y) {
    if (x == y && refl_[i]) return false;                  // R1
    return covered(i, x, y);
}

void Relaxation::box(AgentIdx i, std::size_t psi, bool negate, Word* T, Word* F) {
    if (!sigs_ready_) build_sigs();
    const Word* pT = arena_.data() + psi + (negate ? rw_ : 0);
    const Word* pF = arena_.data() + psi + (negate ? 0 : rw_);
    Word* base = scratch_.data() + 4 * std::size_t(rw_);
    copy_row(base, elim_.data(), rw_);
    or_row(base, pT, rw_);
    std::fill(T, T + rw_, 0);
    std::fill(F, F + rw_, 0);
    tcache_.clear();
    fset_.assign(s_.num_sets(), -1);
    for (WorldIdx x = 0; x < n_; ++x) {
        const std::uint32_t sid = s_.succ_set(i, x);
        if (fset_[sid] < 0) {
            fset_[sid] = 0;
            for (WorldIdx y : s_.set(sid)) if (test_row(pF, y)) { fset_[sid] = 1; break; }
        }
        if (fset_[sid]) set_row(F, x);
        // Worlds of the set failing base ∪ cut row; by R1 a self edge of a
        // reflexive agent also needs x ∈ base, which the cut row cannot excuse.
        // Shared rows first: U(sig) ⊆ cut_row_x, so a world that passes with the
        // shared row passes with its own; only failures need the R4 row of x.
        const std::uint64_t key = (std::uint64_t(sid) << 32) | sig_of_[x];
        auto [it, fresh] = tcache_.try_emplace(key, -1);
        if (fresh) {
            const Word* U = cut_row(i, sig_of_[x]);
            for (WorldIdx y : s_.set(sid))
                if (!test_row(base, y) && !test_row(U, y)) {
                    if (it->second != -1) { it->second = -2; break; }
                    it->second = y;
                }
        }
        std::int64_t bad = it->second;
        if (bad != -1 && public_) {
            const Word* Ux = cut_row_x(i, x);
            bad = -1;
            for (WorldIdx y : s_.set(sid))
                if (!test_row(base, y) && !test_row(Ux, y)) {
                    if (bad != -1) { bad = -2; break; }
                    bad = y;
                }
        }
        const bool self_ok = !refl_[i] || test_row(base, x);
        if (bad == -1 ? self_ok : (bad >= 0 && refl_[i] && WorldIdx(bad) == x && test_row(base, x)))
            set_row(T, x);
    }
}

void Relaxation::common(const std::vector<AgentIdx>& grp, std::size_t psi, Word* T, Word* F) {
    if (!sigs_ready_) build_sigs();
    const BS pT(arena_.data() + psi, arena_.data() + psi + rw_);
    const BS pF(arena_.data() + psi + rw_, arena_.data() + psi + 2 * rw_);
    // F: least fixpoint over original edges.
    BS Y(rw_, 0);
    for (bool changed = true; changed;) {
        changed = false;
        BS Z = pF;
        or_row(Z.data(), Y.data(), rw_);
        for (WorldIdx x = 0; x < n_; ++x) {
            if (test_row(Y.data(), x)) continue;
            bool hit = false;
            for (AgentIdx g : grp) {
                for (WorldIdx y : s_.succ(g, x)) if (test_row(Z.data(), y)) { hit = true; break; }
                if (hit) break;
            }
            if (hit) { set_row(Y.data(), x); changed = true; }
        }
    }
    copy_row(F, Y.data(), rw_);
    // T: greatest fixpoint over guaranteed edges.
    std::size_t edges = 0;
    for (AgentIdx g : grp) for (WorldIdx x = 0; x < n_; ++x) edges += s_.succ(g, x).size();
    if (edges > 50'000'000) { ++ck_fallbacks; copy_row(T, all_.data(), rw_); return; }
    std::vector<std::vector<WorldIdx>> gs(n_);
    for (WorldIdx x = 0; x < n_; ++x)
        for (AgentIdx g : grp)
            for (WorldIdx y : s_.succ(g, x))
                if (!test_row(elim_.data(), y) && ((y == x && refl_[g]) || !covered(g, x, y)))
                    gs[x].push_back(y);
    BS X = all_;
    for (bool changed = true; changed;) {
        changed = false;
        for (WorldIdx x = 0; x < n_; ++x) {
            if (!test_row(X.data(), x)) continue;
            for (WorldIdx y : gs[x])
                if (!test_row(pT.data(), y) || !test_row(X.data(), y)) { X[x / 64] &= ~(Word{1} << (x % 64)); changed = true; break; }
        }
    }
    copy_row(T, X.data(), rw_);
}

TV Relaxation::eval(const Formula& f) {
    std::size_t off;
    {
        // Evaluate, then read the slot: the arena may grow while evaluating.
        struct Rec {
            Relaxation& r;
            std::size_t go(const Formula& f) {
                if (f.id >= r.slot_.size()) { r.slot_.resize(f.id + 1, 0); r.stamp_.resize(f.id + 1, 0); }
                if (r.stamp_[f.id] == r.epoch_) return r.slot_[f.id];
                const std::uint32_t rw = r.rw_;
                std::size_t o = 0;
                auto T = [&](std::size_t x) { return r.arena_.data() + x; };
                auto F = [&](std::size_t x) { return r.arena_.data() + x + rw; };
                switch (f.kind) {
                case FormulaKind::Top: o = r.alloc(); copy_row(T(o), r.all_.data(), rw); break;
                case FormulaKind::Bot: o = r.alloc(); copy_row(F(o), r.all_.data(), rw); break;
                case FormulaKind::Atom:
                    o = r.alloc();
                    copy_row(T(o), r.row(r.canT_, f.atom), rw);
                    copy_row(F(o), r.row(r.canF_, f.atom), rw);
                    break;
                case FormulaKind::Not: {
                    const std::size_t c = go(*f.children[0]);
                    o = r.alloc();
                    copy_row(T(o), F(c), rw); copy_row(F(o), T(c), rw);
                    break;
                }
                case FormulaKind::And:
                case FormulaKind::Or: {
                    for (const auto& c : f.children) go(*c);
                    o = r.alloc();
                    const bool conj = f.kind == FormulaKind::And;
                    copy_row(conj ? T(o) : F(o), r.all_.data(), rw);
                    for (const auto& c : f.children) {
                        const std::size_t k = go(*c);
                        if (conj) { and_row(T(o), T(k), rw); or_row(F(o), F(k), rw); }
                        else      { or_row(T(o), T(k), rw);  and_row(F(o), F(k), rw); }
                    }
                    break;
                }
                case FormulaKind::Belief: {
                    const std::size_t c = go(*f.children[0]);
                    o = r.alloc();
                    r.box(f.agent, c, false, T(o), F(o));
                    break;
                }
                case FormulaKind::Kw: {
                    const std::size_t c = go(*f.children[0]);
                    o = r.alloc();
                    Word* s = r.scratch_.data();
                    r.box(f.agent, c, false, s, s + rw);
                    r.box(f.agent, c, true, s + 2 * rw, s + 3 * rw);
                    copy_row(T(o), s, rw); or_row(T(o), s + 2 * rw, rw);
                    copy_row(F(o), s + rw, rw); and_row(F(o), s + 3 * rw, rw);
                    break;
                }
                case FormulaKind::Common: {
                    const std::size_t c = go(*f.children[0]);
                    o = r.alloc();
                    r.common(f.group, c, T(o), F(o));
                    break;
                }
                }
                r.slot_[f.id] = static_cast<std::uint32_t>(o);
                r.stamp_[f.id] = r.epoch_;
                return o;
            }
        } rec{*this};
        off = rec.go(f);
    }
    return {Span(arena_.data() + off, rw_), Span(arena_.data() + off + rw_, rw_)};
}

bool Relaxation::goal_reached() { return reached(*t_.goal); }

bool Relaxation::reached(const Formula& f) {
    const TV g = eval(f);
    bool some = false;
    for (std::uint32_t k = 0; k < rw_; ++k) {
        if (desig_[k] & g.T[k]) some = true;
        if (desig_[k] & ~(g.T[k] | elimD_[k])) return false;
    }
    return some;
}

bool Relaxation::step() {
    if (std::chrono::steady_clock::now() > until) { timed_out = true; return false; }
    BS nT = canT_, nF = canF_, nelim = elim_, nelimD = elimD_, ngT = genT_, ngF = genF_;
    std::vector<std::uint8_t> napp = app_, ntype = type_;
    BS allfail(rw_), tmp(rw_);
    const std::size_t G = t_.num_agents();
    for (std::size_t a = 0; a < t_.actions.size(); ++a) {
        const Action& A = t_.actions[a];
        const std::uint32_t b = ev_base_[a];
        for (std::size_t e = 0; e < A.events.size(); ++e) {
            const TV p = eval(*A.events[e].precondition);
            or_row(row(ngT, b + e), p.T.data(), rw_);
            or_row(row(ngF, b + e), p.F.data(), rw_);
        }
        bool app = napp[a];
        for (EventIdx e : A.designated_events)
            for (std::uint32_t k = 0; k < rw_ && !app; ++k)
                if (desig_[k] & row(ngT, b + e)[k]) app = true;
        napp[a] = app;
        if (!app) continue;
        for (AgentIdx i = 0; i < G; ++i) {
            const std::uint32_t tb = type_base_[a * G + i];
            ntype[tb] = 1;
            for (std::size_t c = 0; i < A.obs_cases.size() && c < A.obs_cases[i].size(); ++c) {
                const TV v = eval(*A.obs_cases[i][c].condition);
                for (std::uint32_t k = 0; k < rw_; ++k) if (v.T[k] & desig_[k]) { ntype[tb + 1 + c] = 1; break; }
            }
        }
        copy_row(allfail.data(), all_.data(), rw_);
        for (std::size_t e = 0; e < A.events.size(); ++e) {
            const Event& ev = A.events[e];
            and_row(allfail.data(), row(ngF, b + e), rw_);
            for (const auto& [p, phi] : ev.post_true) {
                const TV v = eval(*phi);
                for (std::uint32_t k = 0; k < rw_; ++k) row(nT, p)[k] |= v.T[k] & row(ngT, b + e)[k];
            }
            for (const auto& [p, phi] : ev.post_false) {
                const TV v = eval(*phi);
                for (std::uint32_t k = 0; k < rw_; ++k) row(nF, p)[k] |= v.T[k] & row(ngT, b + e)[k];
            }
        }
        // R3: with one designated event e* whose precondition is i-introspective
        // (K45 for i), applicability makes pre(e*) true at every designated
        // world, hence at their i-successors. A world y reached from a surely
        // designated d by a surely surviving i-edge keeps the descendant (v, e*).
        if (A.designated_events.size() == 1) {
            const EventIdx es = *A.designated_events.begin();
            for (AgentIdx i = 0; i < G; ++i) {
                if (!intro_[std::size_t(b + es) * G + i]) continue;
                for (WorldIdx d = 0; d < n_; ++d) {
                    if (!test_row(desig_.data(), d) || test_row(elimD_.data(), d)) continue;
                    for (WorldIdx y : s_.succ(i, d))
                        if (test_row(allfail.data(), y) && !test_row(elim_.data(), y) && !possibly_cut(i, d, y))
                            allfail[y / 64] &= ~(Word{1} << (y % 64));
                }
            }
        }
        // Applicability gives every designated world a designated descendant, so
        // a designated world that surely stays designated also stays alive.
        for (std::uint32_t k = 0; k < rw_; ++k) allfail[k] &= ~(desig_[k] & ~elimD_[k]);
        or_row(nelim.data(), allfail.data(), rw_);
        // Only a branch of an action with several designated events can drop a
        // designated world; with one designated event, applicability keeps it.
        if (A.designated_events.size() > 1)
            for (EventIdx e : A.designated_events)
                for (std::uint32_t k = 0; k < rw_; ++k) nelimD[k] |= row(ngF, b + e)[k] & desig_[k];
    }
    for (std::uint32_t k = 0; k < rw_; ++k) nelimD[k] |= nelim[k] & desig_[k];

    const bool changed = nT != canT_ || nF != canF_ || nelim != elim_ || nelimD != elimD_ ||
                         ngT != genT_ || ngF != genF_ || napp != app_ || ntype != type_;
    canT_.swap(nT); canF_.swap(nF); elim_.swap(nelim); elimD_.swap(nelimD);
    genT_.swap(ngT); genF_.swap(ngF); app_.swap(napp); type_.swap(ntype);
    ++layer_;
    arena_.clear();
    ++epoch_;
    sigs_ready_ = false;
    return changed;
}

std::uint32_t h_delta(const PlanningTask& t, const EpistemicState& s, std::uint64_t* layers_run) {
    Relaxation r(t, s);
    std::uint32_t h = kInf;
    for (;;) {
        if (r.goal_reached()) { h = r.layer(); break; }
        if (r.layer() >= r.max_layers || !r.step()) break;
    }
    if (layers_run) *layers_run = r.layer();
    return h;
}

std::optional<std::uint32_t> h_delta_until(const PlanningTask& t, const EpistemicState& s,
                                           std::chrono::steady_clock::time_point until) {
    Relaxation r(t, s);
    r.until = until;
    for (;;) {
        if (r.goal_reached()) return r.layer();
        if (r.layer() >= r.max_layers) return kInf;
        if (!r.step()) return r.timed_out ? std::nullopt : std::optional<std::uint32_t>(kInf);
    }
}

bool prunes(const PlanningTask& t, const EpistemicState& s, float h) {
    switch (t.dead_end_check) {
    case DeadEndCheck::Off:
    case DeadEndCheck::Root:
        return false;
    case DeadEndCheck::Suspect:
        if (std::isinf(h)) return true;
        if (h < t.dead_end_suspect) return false;
        return h_delta(t, s) == kInf;
    case DeadEndCheck::All:
        return std::isinf(h) || h_delta(t, s) == kInf;
    }
    return false;
}

float DistinguishabilityHeuristic::operator()(const EpistemicState& s, const PlanningTask& task) const {
    const std::uint32_t v = sum_ ? h_delta_both(task, s).sum : h_delta(task, s);
    return v == kInf ? std::numeric_limits<float>::infinity() : float(v);
}

Estimates h_delta_both(const PlanningTask& t, const EpistemicState& s) {
    Relaxation r(t, s);
    std::vector<const Formula*> cs;
    if (t.goal->kind == FormulaKind::And) for (const auto& c : t.goal->children) cs.push_back(c.get());
    else cs.push_back(t.goal.get());
    std::vector<std::uint32_t> first(cs.size(), kInf);
    Estimates out;
    std::size_t open = cs.size();
    for (;;) {
        if (out.h == kInf && r.goal_reached()) out.h = r.layer();
        for (std::size_t k = 0; k < cs.size(); ++k)
            if (first[k] == kInf && r.reached(*cs[k])) { first[k] = r.layer(); --open; }
        if ((out.h != kInf && open == 0) || r.layer() >= r.max_layers || !r.step()) break;
    }
    out.layers = r.layer();
    if (open == 0) { out.sum = 0; for (auto l : first) out.sum += l; }
    return out;
}

} // namespace hdelta
