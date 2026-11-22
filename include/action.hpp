#pragma once
#include "bitset.hpp"
#include "types.hpp"
#include "formula.hpp"

// One event inside an action's event model
struct Event {
    EventIdx id;
    std::string name;

    FormulaPtr precondition;   // must hold in world for (world,event) to exist

    // postconditions: atom -> formula that gives new truth value
    // if atom not in map -> unchanged
    std::unordered_map<AtomIdx, FormulaPtr> post_true;   // atom becomes true if formula holds
    std::unordered_map<AtomIdx, FormulaPtr> post_false;  // atom becomes false if formula holds

    bool is_nil{false};        // trivial nil event (no effects)
};

// One conditional observability case for an agent:
// if condition holds at world w, use this event relation.
// Cases are evaluated in order; first match wins.
struct ObsCase {
    FormulaPtr condition;
    std::vector<std::unordered_set<EventIdx>> relation; // [event_id] -> reachable event ids

    // relation as bit rows: bit f of row e set iff f ∈ relation[e].
    std::vector<bits::Word> relation_bits;
    std::uint32_t           relation_words{0};

    void finalize(std::size_t num_events) {
        relation_words = static_cast<std::uint32_t>(bits::words_for(num_events));
        relation_bits.assign(num_events * relation_words, 0);
        for (std::size_t e = 0; e < relation.size() && e < num_events; ++e)
            for (EventIdx f : relation[e])
                if (f < num_events)
                    bits::set(event_row(static_cast<EventIdx>(e)), f);
    }

    [[nodiscard]] bits::WordSpan event_row(EventIdx e) noexcept
        { return {relation_bits.data() + std::size_t(e) * relation_words, relation_words}; }
    [[nodiscard]] bits::ConstWordSpan event_row(EventIdx e) const noexcept
        { return {relation_bits.data() + std::size_t(e) * relation_words, relation_words}; }
};

// Abstract epistemic action = event model + observability
struct Action {
    std::string name;

    std::vector<Event> events;
    std::unordered_set<EventIdx> designated_events;   // E_d

    // obs_cases[agent_idx] = ordered list of (condition, event_relation) pairs.
    // Evaluated per world during product update; first matching case is used.
    // If no case matches, agent is treated as fully observable.
    std::vector<std::vector<ObsCase>> obs_cases;

    // Relation of the first declared observability type, used for an agent
    // none of whose conditions holds at every designated world (as in plank).
    ObsCase default_obs;

    size_t num_agents{0};

    // True iff the action has exactly one designated event and is therefore
    // an ontic action (no sensing branches). Sensing actions have |E_d| >= 2
    // (the second designated event is typically the nil / no-op branch).
    bool is_ontic() const { return designated_events.size() == 1; }

    // Strong applicability (conformant semantics):
    //   - For ontic actions: ALL designated worlds must satisfy the precondition
    //     of every designated event. An ontic action that fires in only some
    //     actual worlds would silently drop the non-firing worlds from the
    //     designated set, producing a state that conflates partial execution
    //     with full execution and gives spurious goal satisfaction.
    //   - For sensing actions: every designated world satisfies the
    //     precondition of at least one designated event, so whichever world is
    //     actual, some outcome can occur. Sensing actions branch via
    //     product_update_split, so worlds where one event fires and worlds
    //     where another fires land in separate subtrees. (An existential check
    //     would drop the designated worlds where no event fires, conflating
    //     "cannot run here" with "ran and observed".)
    bool applicable(const EpistemicState& s) const;

    // Weak applicability: existential check used only for AO* action ranking
    // (heuristic ordering). Never used to decide whether to generate a successor.
    bool applicable_weak(const EpistemicState& s) const;
};