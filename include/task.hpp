#pragma once
#include "types.hpp"
#include "state.hpp"
#include "action.hpp"

struct AgentSymmetry;

struct PlanningTask {
    // Language
    std::vector<std::string> atom_names;
    std::vector<std::string> agent_names;

    // Task components
    EpistemicState            init;
    std::vector<Action>       actions;
    FormulaPtr                goal;

    // Frame semantics: true = KD45 (belief/doxastic), false = S5 (knowledge).
    bool kd45 = false;

    // Delete non-serial worlds after each KD45 update (belief expansion with
    // repair). Off by default: plank, the IεPC validator, does not repair, and
    // repair rejects belief-contradicting events such as deception.
    bool kd45_repair = false;

    // GBFS expands the heuristic's preferred actions first.
    bool helpful_actions = true;

    [[nodiscard]] bool repair_seriality() const noexcept { return kd45 && kd45_repair; }

    // S5 refuses a product that left the frame, KD45 repairs one if asked to,
    // and an unrepaired KD45 task keeps the behaviour it always had.
    [[nodiscard]] FrameGuard frame_guard() const noexcept
    {
        if (!kd45) return FrameGuard::Refuse;
        return kd45_repair ? FrameGuard::Prune : FrameGuard::None;
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