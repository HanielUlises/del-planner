#pragma once
#include "heuristic.hpp"

#include <cstdint>
#include <unordered_map>
#include <vector>

// kadd: additive heuristic over a delete-free relaxation of knowledge.
//
// Facts are literals, knowledge literals [j]ℓ, and the modal formulas the goal
// and preconditions mention; Kw_j p is derived from [j]p ∨ [j]¬p. Relaxed
// operators come from the event models:
//
//   - an unconditional postcondition of event e adds its literal;
//   - agent j, in an observability case whose event row from e contains only
//     events after which literal ℓ holds (it is a precondition conjunct not
//     overwritten, or a postcondition), gains [j]ℓ under pre(e) ∧ case condition.
//
// So seeing an event fully teaches its preconditions, seeing it partially
// teaches only what all indistinguishable events share, and not seeing it
// (nil) teaches nothing. A fact holds initially if it holds at some designated
// world, which keeps the estimate meaningful inside sensing branches.
//
// Preconditions compile to one And/Or DAG shared across actions through formula
// interning; facts and operators that cannot reach the goal are dropped.
class KnowledgeRelaxationHeuristic : public Heuristic {
public:
    // Add sums goal-conjunct costs (kadd); FF counts the operators of the
    // relaxed plan extracted from the same fixpoint (kff).
    enum class Estimate : std::uint8_t { Add, FF };

    explicit KnowledgeRelaxationHeuristic(const PlanningTask& task, Estimate estimate = Estimate::Add);

    float operator()(const EpistemicState& s, const PlanningTask& task) const override;

    struct Analysis {
        std::uint32_t rounds{0};          // fixpoint rounds
        std::uint32_t depth{0};           // largest finite goal-conjunct cost
        std::uint32_t dead_goals{0};      // goal conjuncts unreachable in the relaxation
        std::uint32_t relaxed_plan{0};    // operators in the extracted relaxed plan
        std::uint32_t facts{0}, operators{0};
    };
    [[nodiscard]] Analysis analyse(const EpistemicState& s) const;

    // Applicable actions of a relaxed plan extracted from the cost fixpoint.
    bool preferred(const EpistemicState& s, const PlanningTask& task,
                   std::vector<ActionIdx>& out) const override;

    // The same estimates from the facts that hold initially, given as one flag
    // per fact formula: fact f holds iff fact_formulas()[f] is true at some
    // designated world (null entries are never initial). The symbolic search
    // computes the flags on BDDs and shares everything else.
    [[nodiscard]] const std::vector<FormulaPtr>& fact_formulas() const noexcept { return facts_; }
    [[nodiscard]] std::vector<std::uint8_t> initial_facts(const EpistemicState& s) const;
    [[nodiscard]] float estimate(const std::vector<std::uint8_t>& init) const;
    bool preferred_from(const std::vector<std::uint8_t>& init, std::vector<ActionIdx>& out) const;

private:
    enum class Kind : std::uint8_t { True, False, Fact, And, Or };

    struct Req {
        Kind          kind{Kind::True};
        std::uint32_t fact{0};             // Kind::Fact
        std::uint32_t begin{0}, end{0};    // children, into req_children_
    };

    struct Op {
        std::uint32_t pre{0};              // requirement
        std::uint32_t begin{0}, end{0};    // added facts, into op_adds_
        ActionIdx     action{0};
    };

    std::uint32_t compile(const FormulaPtr& f, bool negated);
    std::uint32_t fact(const FormulaPtr& f);
    std::uint32_t node(Kind k, std::vector<std::uint32_t> children);
    void          add_op(std::uint32_t pre, const std::vector<std::uint32_t>& adds);
    void          prune();
    // Cost fixpoint at s; supporter[f] is the operator giving fact f its cost,
    // or -1 when initial or derived.
    std::uint32_t costs(const EpistemicState& s, std::vector<std::int32_t>& fc,
                        std::vector<std::int32_t>& rc, std::vector<std::int32_t>* supporter) const;
    std::uint32_t costs(const std::vector<std::uint8_t>& init, std::vector<std::int32_t>& fc,
                        std::vector<std::int32_t>& rc, std::vector<std::int32_t>* supporter) const;
    // Relaxed plan backchained from the goal; returns its operator count and
    // appends to `helpful` the actions of operators applicable now.
    std::uint32_t extract(const std::vector<std::int32_t>& rc, const std::vector<std::int32_t>& sup,
                          std::vector<ActionIdx>* helpful) const;

    Estimate      estimate_{Estimate::Add};
    ActionIdx     current_action_{0};

    std::vector<Req>           reqs_;
    std::vector<std::uint32_t> req_children_;
    std::unordered_map<std::uint64_t, std::uint32_t> compiled_;   // (formula id, polarity)

    std::vector<FormulaPtr>    facts_;
    std::unordered_map<std::uint32_t, std::uint32_t> fact_of_;    // formula id → fact
    std::vector<std::int32_t>  derived_;                           // fact → requirement, or -1

    std::vector<Op>            ops_;
    std::vector<std::uint32_t> op_adds_;

    std::vector<std::uint32_t> goal_;                              // one requirement per conjunct
};
