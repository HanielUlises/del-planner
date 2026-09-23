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

// What to do with a product that leaves the declared frame, which here means a
// world where some agent has no successor at all. Such an agent believes
// everything, and a goal about what it knows is then satisfied by a model that
// says nothing.
//
// KD45 loses seriality legitimately --- belief expansion against what an agent
// believed is the point of the frame --- so the answer there is to delete the
// worlds that lost it, which is what the repair has always done. S5 does not:
// an equivalence relation is serial, so a state that is not cannot have come
// from a sound update, and deleting worlds would hide the defect behind a
// smaller model that still answers questions. So S5 refuses the successor.
enum class FrameGuard : std::uint8_t {
    None = 0,   // take the product as it comes
    Prune,      // delete the worlds that lost seriality (KD45 repair)
    Refuse,     // no successor at all (S5)
};

// Forward declarations
struct Formula;
struct Action;
struct EpistemicState;
using FormulaPtr = std::shared_ptr<Formula>;
