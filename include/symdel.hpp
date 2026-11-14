#pragma once
// Symbolic DEL states for plank's semantics: product update and bisimulation
// contraction on BDDs, without enumerating worlds.
//
// A state S = (H, theta, delta, Omega_1..Omega_n) over atoms P and hidden slots H:
//   theta(x)      the worlds (one satisfying assignment of x_P, x_H per world)
//   delta(x)      the designated worlds
//   Omega_i(x,y)  R_i, restricted to theta x theta
// Its denotation [[S]] is the explicit model on the assignments of theta.
//
//   sat(S, phi)        = the extension of phi in [[S]]                    (Lemma 1)
//   [[update(S, a)]]   ≅ [[S]] ⊗ a, plank's product update               (Lemma 2)
//   [[contract(S)]]    ≅ the bisimulation quotient of [[S]] restricted
//                        to the worlds reachable from delta              (Lemma 3)
//
// Variables: every atom and every hidden slot has three copies x, y, u (u is
// used only inside the bisimulation fixpoint). Contracted states use hidden
// slots 0..m-1, so states that coexist in a search agree on every variable.
#include "bdd.hpp"
#include "action.hpp"
#include "formula.hpp"
#include "state.hpp"
#include "task.hpp"

#include <unordered_map>
#include <vector>

namespace sym {

using bdd::Ref;

struct SymState {
    std::vector<std::uint32_t> hid;     // hidden slots in use (0..m-1 when normalised)
    Ref theta{bdd::kFalse}, delta{bdd::kFalse};
    std::vector<Ref> om;
    std::unordered_map<std::uint32_t, Ref> memo;   // sat(), by formula id
};

class Engine {
public:
    static constexpr std::uint32_t kMaxHidden = 512;

    explicit Engine(const PlanningTask& t, bool hidden_on_top = true);

    bdd::Manager& mgr() { return m_; }

    // Model checking
    Ref sat(SymState& s, const Formula& f);
    bool holds(SymState& s, const Formula& f);
    bool applicable(SymState& s, const Action& a);

    // Construction and the two operations
    SymState from_explicit(const EpistemicState& e);
    // The initial state of a finitary S5 theory, without enumerating worlds:
    // theta = fixed atoms ∧ ⋀ type-2, delta = theta ∧ ⋀ type-1,
    // Omega_i = theta(x) ∧ theta(y) ∧ ⋀_{φ ∈ type-3(i)} (φ(x) <-> φ(y)).
    SymState from_theory(const InitialTheory& th);
    SymState update(SymState& s, const Action& a);
    SymState contract(const SymState& s);

    // Measurement and testing
    double worlds(const SymState& s) { return m_.count(s.theta, xvars(s.hid)); }
    double designated(const SymState& s) { return m_.count(s.delta, xvars(s.hid)); }
    std::vector<std::uint32_t> xvars(const std::vector<std::uint32_t>& hid, bool atoms = true) const {
        std::vector<std::uint32_t> v;
        if (atoms) for (std::uint32_t p = 0; p < na_; ++p) v.push_back(av(p, 0));
        for (std::uint32_t k : hid) v.push_back(hv(k, 0));
        return v;
    }
    std::size_t nodes(const SymState& s) const;
    EpistemicState explicit_state(const SymState& s);   // enumerate [[S]]

    std::size_t reencode_limit = 4096;

private:
    std::uint32_t av(std::uint32_t p, unsigned c) const   // atom p, copy c
        { return htop_ ? 3 * kMaxHidden + 3 * p + c : 3 * p + c; }
    std::uint32_t hv(std::uint32_t k, unsigned c) const   // hidden slot k, copy c
        { return htop_ ? 3 * k + c : 3 * na_ + 3 * k + c; }

    Ref box(SymState& s, std::uint32_t ag, Ref ext);
    Ref prop(const Formula& f, unsigned c);   // propositional formula over copy c
    Ref vars(const std::vector<std::uint32_t>& hid, unsigned c, bool atoms = true);
    Ref code(const std::vector<std::uint32_t>& slots, std::uint32_t value, unsigned c);
    std::vector<std::uint32_t> copy_map(std::initializer_list<std::pair<unsigned, unsigned>> m) const;

    const PlanningTask& t_;
    std::uint32_t na_, ng_;
    bool htop_;
    bdd::Manager m_;
    // Renamings over all variables (unused variables map harmlessly).
    std::vector<std::uint32_t> XY_, YX_, YU_, SWAP_, XY_UX_, X_U_;
    Ref eqP_{bdd::kTrue};        // ∧_p x_p <-> y_p
};

} // namespace sym
