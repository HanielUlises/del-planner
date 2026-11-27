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
#include "symmetry.hpp"
#include "task.hpp"

#include <chrono>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sym {

using bdd::Ref;

struct SymState {
    std::vector<std::uint32_t> hid;     // hidden slots in use (0..m-1 when normalised)
    Ref theta{bdd::kFalse}, delta{bdd::kFalse};
    std::vector<Ref> om;
    std::unordered_map<std::uint32_t, Ref> memo;   // sat(), by formula id
};

struct EngineStats {
    std::size_t contractions{0};
    std::size_t fixpoint_skipped{0};     // contractions with H = ∅ after projection
    std::size_t fixpoint_runs{0};        // contractions that ran the E fixpoint
    std::size_t fixpoint_rounds{0};
    std::size_t equivalence_checks{0};   // bisimilarity tests on disjoint unions
    std::size_t equivalent_by_relabel{0};   // ... settled by a hidden relabelling
    std::size_t gc_freed{0};
    std::size_t box_hits{0};             // [i]φ reused from another state
    std::size_t fixes_same{0}, fixes_relabel{0}, fixes_invariant{0}, fixes_equivalent{0}, fixes_rejected{0};   // H != 0 swap tests
    double t_reach{0}, t_fixpoint{0}, t_quotient{0};   // contraction phases, seconds
    double t_certificate{0};
    std::size_t certified{0};            // H != 0 contractions proved minimal without the fixpoint
};

class Relaxation;

struct PhaseTimer {
    double& acc;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    ~PhaseTimer() { acc += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }
};

class Engine {
public:
    static constexpr std::uint32_t kMaxHidden = 512;
    friend class Relaxation;

struct PhaseTimer {
    double& acc;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    ~PhaseTimer() { acc += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }
};

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

    // Garbage collection. Every BDD a caller still needs must be a root: the
    // engine adds its own; sat() memos of states not listed are dangling after.
    static void roots_of(const SymState& s, std::vector<Ref>& out);
    std::size_t gc(std::vector<Ref> roots);

    // Identity of contracted (normalised) states.
    //   fingerprint  structural 128-bit digest of (|H|, θ, δ, Ω): equal for
    //                equal encodings, which for H = ∅ are canonical, so there it
    //                decides bisimilarity (up to hash collisions)
    //   invariant    digest of the projections ∃H.θ, ∃H.δ, ∃H.Ω_i: equal for
    //                bisimilar states whatever their hidden encoding
    //   equivalent   exact: bisimilarity on the disjoint union (same fixpoint
    //                as contraction), for states whose invariants agree
    Fingerprint fingerprint(const SymState& s);
    Fingerprint invariant(const SymState& s);
    bool equivalent(const SymState& a, const SymState& b);

    // True when renaming agents by the swap (and atoms by sw.atom) maps s onto
    // itself, up to bisimilarity: node equality for H = ∅, the invariant and
    // then equivalent() otherwise.
    bool fixes(const SymState& s, const AgentSymmetry::Swap& sw);
    // Sufficient for a ≅ b: a bijection of hidden values maps a's encoding
    // onto b's. False means "not shown".
    bool relabels_onto(const SymState& a, const SymState& b);

    // Necessary conditions for fixes(), computed once per state: edges per
    // agent and worlds per atom. A swap that fixes s preserves both.
    struct SwapFilter {
        std::vector<double> edges, atoms;
        [[nodiscard]] bool admits(const AgentSymmetry::Swap& sw) const {
            if (edges[sw.a] != edges[sw.b]) return false;
            for (std::size_t p = 0; p < atoms.size(); ++p) if (atoms[p] != atoms[sw.atom[p]]) return false;
            return true;
        }
    };
    SwapFilter swap_filter(const SymState& s);

    std::size_t reencode_limit = 4096;
    // When the fixpoint runs: by the pairwise relation E(x,y) (default) or by
    // signature refinement over block numbers (env SYM_SIGNATURE=1; measured
    // slower on intermediate gossip, where E stays small).
    bool signature_refinement = false;
    // Skip the fixpoint when worlds are told apart at depth one (env SYM_NOCERT=1 disables).
    bool minimality_certificate = true;
    EngineStats stats;

private:
    // Projects away hidden slots determined by the other variables on θ (an
    // isomorphism); afterwards H = ∅ iff the model is valuation-injective.
    void drop_determined(SymState& q);
    // Maximal bisimulation E(x, y) of the model, designation counted as an atom.
    Ref bisimulation(const std::vector<std::uint32_t>& hid, Ref theta, Ref delta,
                     const std::vector<Ref>& om, Ref E_init = bdd::kFalse);
    // Pairs of D that agree at depth one: for every agent, the same sets of
    // (valuation, designated?) among their successors.
    Ref depth_one_pairs(const std::vector<std::uint32_t>& hid, Ref theta, Ref delta,
                        const std::vector<Ref>& om, Ref D);
    Ref same_world(const std::vector<std::uint32_t>& hid, Ref theta);   // identity on theta
    SymState refine_quotient(const SymState& r);
    SymState pairwise_quotient(const SymState& r, std::optional<PhaseTimer>& phase, Ref E_init);
    SymState finish_quotient(SymState q);
    SymState quotient_by(const SymState& r, Ref E);   // representatives of E's classes
    // Variables below the three copies, used only by refine_quotient: agent
    // code a, a copy z of the atoms, block bits l.
    static constexpr std::uint32_t kBlockBits = 32;
    std::uint32_t base_{0}, abits_{0};
    std::uint32_t zv(std::uint32_t p) const { return base_ + abits_ + p; }
    std::uint32_t lv(std::uint32_t j) const { return base_ + abits_ + na_ + j; }
    std::pair<std::uint64_t, std::uint64_t> shash(Ref r);

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
    std::vector<std::uint32_t> XY_, YX_, YU_, SWAP_, XY_UX_, X_U_, XY_YU_, UX_, UY_;
    std::vector<std::uint8_t> XYPAIR_;   // x copies: swap_adjacent transposes x <-> y
    Ref eqP_{bdd::kTrue};        // ∧_p x_p <-> y_p
    std::unordered_map<Ref, std::pair<std::uint64_t, std::uint64_t>> hmemo_;
    struct BoxKey {
        Ref om, ext, theta;
        bool operator==(const BoxKey&) const = default;
    };
    struct BoxHash {
        std::size_t operator()(const BoxKey& k) const noexcept {
            return std::size_t(bits::mix64((std::uint64_t(k.om) << 32 | k.ext) ^ bits::mix64(k.theta)));
        }
    };
    std::unordered_map<BoxKey, Ref, BoxHash> box_memo_;
    std::uint64_t hmemo_epoch_{~std::uint64_t{0}};
};

} // namespace sym
