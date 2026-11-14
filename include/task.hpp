#pragma once
#include "types.hpp"
#include "state.hpp"
#include "action.hpp"

#include <optional>

struct AgentSymmetry;

// An initial state given as a finitary S5 theory, as plank grounds it
// (finitary_s5_theory_grounder):
//   worlds      the valuations satisfying every type-2 formula, with the fixed
//               (fact) atoms at their values;
//   designated  the worlds satisfying every type-1 formula;
//   R_i(w, v)   iff w and v agree on every type-3 formula of agent i.
// All formulas are propositional.
struct InitialTheory {
    std::vector<FormulaPtr>                designated;   // type 1
    std::vector<FormulaPtr>                worlds;       // type 2
    std::vector<std::vector<FormulaPtr>>   observes;     // type 3, per agent
    std::vector<std::pair<AtomIdx, bool>>  fixed;        // fact atoms
};

// Where the searches test for provable dead ends with h^Δ (hdelta.hpp).
enum class DeadEndCheck : std::uint8_t {
    Off,        // never
    Root,       // the initial state only
    Suspect,    // the initial state, and nodes the heuristic reports as relaxation-dead
    All,        // every generated node
};

struct PlanningTask {
    // Language
    std::vector<std::string> atom_names;
    std::vector<std::string> agent_names;

    // Task components
    EpistemicState            init;
    // Set when the task gives the initial state as a finitary S5 theory. `init`
    // is then its explicit materialisation, or empty (0 worlds) when the theory
    // has more valuations than the parser enumerates; the symbolic engine builds
    // its initial state from the theory directly.
    std::optional<InitialTheory> theory;
    std::vector<Action>       actions;
    FormulaPtr                goal;

    // Frame semantics: true = KD45 (belief/doxastic), false = S5 (knowledge).
    bool kd45 = false;

    // Delete non-serial worlds after each KD45 update (belief expansion with
    // repair). Off by default: plank, the IεPC validator, does not repair, and
    // repair rejects belief-contradicting events such as deception.
    bool kd45_repair = false;

    // Refuse every state in which an agent believes a contradiction at a world
    // the designated ones can reach, so that no goal, precondition or
    // observability condition is satisfied for want of a world to check. Off by
    // default: coin-in-the-box's reference plans rely on such states.
    bool consistent_beliefs = false;

    // GBFS expands the heuristic's preferred actions first.
    bool helpful_actions = true;

    // Dead-end detection with h^Δ; Suspect checks nodes whose heuristic value
    // is at least dead_end_suspect, the per-conjunct penalty of kadd and kff.
    DeadEndCheck dead_end_check = DeadEndCheck::Suspect;
    float        dead_end_suspect = 1000.f;

    [[nodiscard]] bool repair_seriality() const noexcept { return kd45 && kd45_repair; }

    // Repair leaves every surviving world serial, so it subsumes the check.
    [[nodiscard]] Seriality seriality() const noexcept
    {
        if (repair_seriality()) return Seriality::Repair;
        return consistent_beliefs ? Seriality::Require : Seriality::Ignore;
    }

    // True iff at least one action has agents with heterogeneous observability
    // (some Fully, some Oblivious or conditional). Set by the parser after all
    // actions are loaded. Gossip, Grapevine, and AMC are the canonical cases.
    // Used by the heuristic selector to prefer knowledge spread over epistemic distance
    // and by strategy selector to avoid routing partial-obs domains to GBFS.
    bool partial_obs = false;

    // True iff the goal is a pure conjunction of Kw (knowing-whether) formulas
    // with no bare atom conjuncts. Set by the parser.
    // Kw-only goals have a specific gradient structure that KnowledgeSpreadHeuristic
    // is designed to exploit; EpistemicDistance wastes cycles on the wrong projection.
    bool goal_kw_only = false;

    // Agent swaps that map the task onto itself; null when none or disabled.
    std::shared_ptr<const AgentSymmetry> symmetry;

    std::unordered_map<std::string, AtomIdx>   atom_index;
    std::unordered_map<std::string, AgentIdx>  agent_index;
    std::unordered_map<std::string, ActionIdx> action_index;

    size_t num_atoms()   const { return atom_names.size(); }
    size_t num_agents()  const { return agent_names.size(); }
    size_t num_actions() const { return actions.size(); }
};