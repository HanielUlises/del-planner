#pragma once
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Atom / agent / world / event indices.
using AtomIdx   = std::uint32_t;
using AgentIdx  = std::uint32_t;
using WorldIdx  = std::uint32_t;
using EventIdx  = std::uint32_t;
using ActionIdx = std::uint32_t;

// Sentinel for "no such world", used by the dense (world, event) product table.
inline constexpr WorldIdx kNoWorld = std::numeric_limits<WorldIdx>::max();

// What a product update does about an agent left with no accessible world.
//
// Such an agent believes a contradiction: [i]φ quantifies over an empty set
// and holds for every φ. It happens without any defect in the update: an agent
// that observes an event its beliefs rule out, such as an announcement from a
// speaker it believes ignorant, has nowhere left to go. A goal or precondition
// about that agent is then satisfied by a model that says nothing.
enum class Seriality : std::uint8_t {
    Ignore,     // keep the product as it comes, as plank and the IεPC do
    Repair,     // delete the worlds that lost it (--kd45-repair)
    Require,    // refuse the product (--consistent-beliefs)
};

// Forward declarations
struct Formula;
struct Action;
struct EpistemicState;
using FormulaPtr = std::shared_ptr<Formula>;
