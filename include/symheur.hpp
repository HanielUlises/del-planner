#pragma once
// Heuristics on symbolic states.
//
// kadd / kff: the knowledge relaxation (knowledge_relaxation.hpp) reads a state
// only through "fact φ holds at some designated world", i.e. δ ∧ sat(φ) ≠ ⊥,
// an existential quantification over the designated worlds. initial_facts()
// computes those flags on BDDs; the cost fixpoint is shared with the explicit
// planner, so both representations give the same value on the same state.
//
// h^Δ: the distinguishability relaxation (hdelta.hpp) with its possibility
// sets as BDDs over the x copy and the possible cuts of agent i as one
// relation over (x, y):
//
//   cut_i(x,y)  = Ω_i(x,y) ∧ ⋁_{a possibly applicable} ⋁_{e ∈ E(a)} genT_e(x) ∧
//                 ⋁_{t possible for i in a} ⋀_{f ∈ Q_t(e)} genF_f(y)
//   T([i]ψ)     = θ ∧ ¬∃y. Ω_i(x,y) ∧ ¬cut_i(x,y) ∧ ¬(elim ∨ T(ψ))(y)
//   F([i]ψ)     = θ ∧ ∃y. Ω_i(x,y) ∧ F(ψ)(y)
//   T(C_G ψ)    = νX. θ ∧ ¬∃y. ⋁_{g∈G} (Ω_g ∧ ¬cut_g)(x,y) ∧ ¬elim(y) ∧ ¬(T(ψ) ∧ X)(y)
//   F(C_G ψ)    = μY. θ ∧ ∃y. ⋁_{g∈G} Ω_g(x,y) ∧ (F(ψ) ∨ Y)(y)
//
// and the layer update of hdelta::Relaxation::step() otherwise unchanged. The
// explicit relaxation's refinements R1–R4 are omitted: each removes possible
// cuts or possible eliminations, so without them every possibility set is a
// superset of the explicit one. The symbolic h^Δ is therefore at most the
// explicit h^Δ, still admissible, and ∞ still proves a dead end.
#include "knowledge_relaxation.hpp"
#include "symdel.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

namespace sym {

std::vector<std::uint8_t> initial_facts(Engine& eng, SymState& s,
                                        const KnowledgeRelaxationHeuristic& h);

class Relaxation {
public:
    static constexpr std::uint32_t kInf = std::numeric_limits<std::uint32_t>::max();

    Relaxation(Engine& eng, const PlanningTask& t, SymState& s);

    bool step();                                   // false at the fixpoint
    [[nodiscard]] bool reached(const Formula& f);
    [[nodiscard]] bool goal_reached() { return reached(*t_.goal); }
    [[nodiscard]] std::uint32_t layer() const noexcept { return layer_; }

    struct TV { Ref T, F; };
    TV eval(const Formula& f);

    std::uint32_t max_layers = 100000;
    // step() returns false once this passes; the result is then unknown.
    std::chrono::steady_clock::time_point until = std::chrono::steady_clock::time_point::max();
    bool timed_out = false;

private:
    Engine& e_;
    bdd::Manager& m_;
    const PlanningTask& t_;
    SymState& s_;
    Ref theta_, delta_, theta_y_, ycube_;
    std::vector<Ref> canT_, canF_;          // [atom]
    Ref elim_{bdd::kFalse}, elimD_{bdd::kFalse};
    std::vector<std::uint32_t> ev_base_;    // [action] → first global event
    std::vector<Ref> genT_, genF_;          // [global event]
    std::vector<std::uint8_t> app_;         // [action]
    std::vector<std::uint32_t> type_base_;  // [action · agents + agent]
    std::vector<std::uint8_t> type_;
    std::uint32_t layer_{0};
    std::unordered_map<std::uint32_t, TV> memo_;
    std::vector<Ref> guar_;                 // [agent] Ω_i ∧ ¬cut_i, per layer
    bool cuts_ready_{false};

    Ref to_y(Ref f);
    const ObsCase& obs(const Action& a, AgentIdx i, std::size_t t) const;
    void build_cuts();
    TV box(AgentIdx i, TV psi);
};

std::uint32_t h_delta(Engine& eng, const PlanningTask& t, SymState& s);
std::optional<std::uint32_t> h_delta_until(Engine& eng, const PlanningTask& t, SymState& s,
                                           std::chrono::steady_clock::time_point until);

// Both estimates from one run, as hdelta::h_delta_both.
struct Estimates { std::uint32_t h{Relaxation::kInf}, sum{Relaxation::kInf}; };
Estimates h_delta_both(Engine& eng, const PlanningTask& t, SymState& s);

} // namespace sym
