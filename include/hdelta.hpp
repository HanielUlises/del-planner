// Distinguishability relaxation h^Δ.
//
// Abstract state over the worlds W of a start state s, all monotone:
//   canT[p], canF[p]  literal p possibly true / false at some descendant of x
//   elim              x possibly has no surviving descendant
//   elimD             x ∈ W_d possibly has no designated descendant
//   cut generators    the previous layer's precondition possibilities and
//                     possible observability types, per action and event
//
// Edge (x,y) of agent i is possibly cut iff some possibly applicable action a,
// possibly selected type t of i in a, and event e with T ∈ pre(e)(x) have
// F ∈ pre(f)(y) for every f ∈ Q_t(e). Cuts are recomputed from the generators
// each layer instead of being stored: all ingredients grow monotonically, so the
// latest generators subsume every earlier cut, and no |W|² table is needed.
//
// Formulas read three-valued possibility sets (T, F ⊆ W):
//   T(□_i ψ) = { x | ∀y ∈ R_i(x): cut_i(x,y) ∨ elim(y) ∨ y ∈ T(ψ) }
//   F(□_i ψ) = { x | ∃y ∈ R_i(x): y ∈ F(ψ) }
//   T(C_G ψ) = greatest X: every guaranteed G-successor (not possibly cut, not
//              possibly eliminated) is in T(ψ) ∩ X
//   F(C_G ψ) = least Y: some original G-successor is in F(ψ) ∪ Y
// Layer k over-approximates every state reachable in ≤ k updates, so h^Δ is
// admissible and h^Δ = ∞ proves that no plan and no policy exists.
// h^Δ(s) is the first layer at which the goal is possibly
// true at some designated world and, at every designated world, possibly true
// or possibly no longer designated; ∞ if the fixpoint is reached first.
#pragma once
#include "heuristic.hpp"
#include "task.hpp"

#include <chrono>
#include <optional>

#include <cstdint>
#include <limits>
#include <span>
#include <unordered_map>
#include <vector>

namespace hdelta {

using BS = std::vector<bits::Word>;
using Span = std::span<const bits::Word>;
inline constexpr std::uint32_t kInf = std::numeric_limits<std::uint32_t>::max();

// Possibility sets of one formula in one layer. The spans point into the
// relaxation's arena and stay valid until the next step().
struct TV { Span T, F; };

class Relaxation {
public:
    Relaxation(const PlanningTask& t, const EpistemicState& s);

    // Advance one layer; returns false once nothing changes (fixpoint).
    bool step();
    [[nodiscard]] bool goal_reached();
    [[nodiscard]] bool reached(const Formula& f);   // the goal condition for f
    [[nodiscard]] std::uint32_t layer() const noexcept { return layer_; }

    // Possibility sets of φ in the current layer (memoised per layer).
    TV eval(const Formula& f);

    // For the Theorem 2 check: is the ancestor edge (x,y) of agent i possibly cut,
    // and which worlds are possibly eliminated (from W, from the designated set).
    [[nodiscard]] bool possibly_cut(AgentIdx i, WorldIdx x, WorldIdx y);
    [[nodiscard]] const BS& elim() const noexcept { return elim_; }
    [[nodiscard]] const BS& elimD() const noexcept { return elimD_; }

    // Largest number of layers before giving up (as ∞) — a safety bound only.
    std::uint32_t max_layers = 100000;
    // Checked once per layer; step() returns false when it has passed, and the
    // caller must then treat the result as unknown.
    std::chrono::steady_clock::time_point until = std::chrono::steady_clock::time_point::max();
    bool timed_out = false;
    std::uint64_t ck_fallbacks = 0;   // C_G evaluations too large, T widened to W

private:
    const PlanningTask& t_;
    const EpistemicState& s_;
    std::uint32_t n_, rw_;
    BS all_, desig_;

    // Flat storage, all indexed by rw_-word rows.
    BS canT_, canF_;                         // [atom]
    BS elim_, elimD_;
    std::vector<std::uint32_t> ev_base_;     // [action] → first global event id
    std::uint32_t num_ev_{0};
    BS genT_, genF_;                         // [global event]
    std::vector<std::uint8_t> app_;          // [action]
    std::vector<std::uint32_t> type_base_;   // [action · agents + agent] → first type slot
    std::vector<std::uint8_t> type_;         // possible types
    std::uint32_t layer_{0};

    // Per-layer memo: formula id → arena offset of (T, F), valid when stamp == epoch.
    BS arena_;
    std::vector<std::uint32_t> slot_, stamp_;
    std::uint32_t epoch_{1};

    // Per layer: global events possibly occurring at each world, interned.
    std::uint32_t sig_words_{0};
    BS sigbits_;                              // [sig id] rows of sig_words_
    std::vector<std::uint32_t> sig_of_;
    bool sigs_ready_{false};
    std::unordered_map<std::uint64_t, std::int64_t> tcache_;   // -1: none bad, -2: ≥2 bad, else the bad world
    // Per layer: worlds y such that an (x,y) edge of agent i is possibly cut, for
    // x with signature sig — the union over the sig's events and i's possible types
    // of ∩_{f ∈ Q_t(e)} F(pre f). Keyed by (i, sig), rows in ucache_.
    std::unordered_map<std::uint64_t, std::size_t> urow_;
    BS ucache_;
    std::vector<std::uint32_t> ev_action_;   // [global event] → action
    // Frame facts that hold in every state reachable from s (checked on s and on
    // every event relation of the agent): reflexive frames never lose a self
    // edge (R1); on K45 frames an event whose precondition only speaks about
    // agent i's own attitudes, and which i observes as itself, cannot separate
    // two worlds i confuses (R2).
    std::vector<std::uint8_t> refl_, k45_;
    std::vector<std::uint8_t> intro_;        // [event · agents + agent]
    // R4: when every event relation of every agent is the identity (public
    // events), an event with an i-introspective precondition cannot cut any
    // edge (x,y) with x R_i y. Such events are kept out of the shared cut rows
    // and added per source world.
    bool public_{false};
    std::vector<std::uint8_t> intro_any_;    // [event]
    BS xrow_, mask_;
    // Per layer: cut_row_x rows by (agent, world), and successor-set bit rows.
    std::unordered_map<std::uint64_t, std::size_t> xrow_at_;
    BS xcache_;
    std::vector<std::uint32_t> setrow_at_;   // [set id] → row + 1, 0 if not built
    BS setrows_;
    [[nodiscard]] const bits::Word* set_row_bits(std::uint32_t sid);
    // R4 groups per (agent i, sig): introspective events merged by the set of
    // agents they are introspective for (bit mask), each group one cut row.
    struct IntroGroup { std::uint64_t agents; std::size_t row; };
    std::unordered_map<std::uint64_t, std::vector<IntroGroup>> igroups_;
    BS gcache_;
    const std::vector<IntroGroup>& intro_groups(AgentIdx i, std::uint32_t sig);
    [[nodiscard]] const bits::Word* brow(std::uint32_t g, AgentIdx i);
    [[nodiscard]] const bits::Word* cut_row_x(AgentIdx i, WorldIdx x);
    std::vector<std::uint32_t> brow_;        // [event · agents + agent] → row in bcache_ + 1, 0 if not built
    BS bcache_;                              // ∪_t ∩_{f ∈ Q_t(e)} F(pre f), per (event, agent)
    [[nodiscard]] const bits::Word* cut_row(AgentIdx i, std::uint32_t sig);
    std::vector<std::int8_t> fset_;
    BS scratch_;

    [[nodiscard]] const bits::Word* row(const BS& b, std::size_t i) const { return b.data() + i * rw_; }
    [[nodiscard]] bits::Word* row(BS& b, std::size_t i) { return b.data() + i * rw_; }
    std::size_t alloc();                      // 2·rw_ words in the arena
    void build_sigs();
    [[nodiscard]] bool covered(AgentIdx i, WorldIdx x, WorldIdx y);
    [[nodiscard]] const ObsCase& obs(const Action& a, AgentIdx i, std::size_t t) const;
    void box(AgentIdx i, std::size_t psi, bool negate, bits::Word* T, bits::Word* F);
    void common(const std::vector<AgentIdx>& g, std::size_t psi, bits::Word* T, bits::Word* F);
};

// h^Δ(s): first goal layer, or kInf when the fixpoint misses the goal.
std::uint32_t h_delta(const PlanningTask& t, const EpistemicState& s,
                      std::uint64_t* layers_run = nullptr);

// Both estimates from one run. `sum` adds, over the top-level conjuncts of the
// goal, the first layer at which each conjunct is reached (inadmissible, like
// h_add); `h` is h^Δ. Layers continue past h until every conjunct is reached or
// the fixpoint is hit, so computing `sum` can cost more layers than `h` alone.
struct Estimates { std::uint32_t h{kInf}, sum{kInf}; std::uint64_t layers{0}; };
Estimates h_delta_both(const PlanningTask& t, const EpistemicState& s);

// h^Δ(s), or nullopt when `until` passes first.
std::optional<std::uint32_t> h_delta_until(const PlanningTask& t, const EpistemicState& s,
                                           std::chrono::steady_clock::time_point until);

// Sound dead-end test for a search node with heuristic value h (Theorem 3:
// h^Δ = ∞ proves that no plan and no policy exists). Governed by
// t.dead_end_check: never under Off and Root; under Suspect only when h carries
// the relaxation-dead penalty of kadd/kff; under All at every node. A value of
// +∞ is taken as proof only from the hd heuristics, the only ones that return it.
bool prunes(const PlanningTask& t, const EpistemicState& s, float h);

// h^Δ as a search heuristic: admissible (Sum = false) or the sum of the first
// layers of the goal's conjuncts (Sum = true). Returns +∞ on proven dead ends.
struct DistinguishabilityHeuristic : Heuristic {
    explicit DistinguishabilityHeuristic(bool sum) : sum_(sum) {}
    float operator()(const EpistemicState& s, const PlanningTask& task) const override;
private:
    bool sum_;
};

} // namespace hdelta
