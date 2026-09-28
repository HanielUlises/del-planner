#pragma once
#include "action.hpp"
#include "outcome.hpp"
#include "state.hpp"
#include "world_cap_policy.hpp"

#include <optional>
#include <vector>

// Result of a DEL product update: the updated state together with the dense
// (world, event) → new-world table used to build it.
//
// The table was previously an unordered_map keyed on a packed uint64. It is
// probed in the innermost loop of the relation construction — once per
// (agent, source world, source event, successor world, successor event) — so a
// hash lookup there dominated the update. The product index space is dense and
// small (|W| · |E|), so a flat vector with a sentinel is both smaller and O(1)
// with no hashing.
//
// After KD45 seriality repair the surviving worlds are compacted so that world
// ids are again 0..|W'|-1; the table is patched in place, and entries whose
// world did not survive are reset to kNoWorld.
struct ProductUpdateResult {
    EpistemicState        state;
    std::vector<WorldIdx> pair_to_idx;    // size |W| · num_events
    std::uint32_t         num_events{0};

    [[nodiscard]] WorldIdx at(WorldIdx w, EventIdx e) const noexcept {
        return pair_to_idx[std::size_t(w) * num_events + e];
    }
};

// An agent with no accessible world at a world that formulas evaluated at W*
// can reach, which is every world reachable from W* along any agent's
// relation. Truth at W* depends on those worlds alone, so a state without one
// satisfies no formula vacuously. Returns the first found, in breadth-first
// order from W*.
struct BeliefCollapse {
    AgentIdx agent;
    WorldIdx world;
};
[[nodiscard]] std::optional<BeliefCollapse> find_collapse(const EpistemicState& s);

// Compute s ⊗ a (DEL product update).
[[nodiscard]] Outcome<ProductUpdateResult>
product_update_with_map(const EpistemicState& s, const Action& a,
                        Seriality seriality = Seriality::Ignore,
                        const WorldCapPolicy& cap = make_world_cap_policy(false));

[[nodiscard]] Outcome<EpistemicState>
product_update(const EpistemicState& s, const Action& a,
               Seriality seriality = Seriality::Ignore,
               const WorldCapPolicy& cap = make_world_cap_policy(false));

// Sensing update: one state per designated event, all sharing the same product
// model and differing only in which worlds are designated.
[[nodiscard]] std::vector<std::pair<EventIdx, EpistemicState>>
product_update_split(const EpistemicState& s, const Action& a,
                     Seriality seriality = Seriality::Ignore,
                     const WorldCapPolicy& cap = make_world_cap_policy(false));
