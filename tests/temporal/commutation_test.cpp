// When two epistemic actions may overlap: tests of temporal::commutes.
//
// Self-contained: no test framework, so the core keeps no dependency beyond
// nlohmann_json. Each TEST is a function; the runner reports every failed
// check and exits non-zero if any failed.

#include "bisimulation.hpp"
#include "parser.hpp"
#include "temporal.hpp"

#include "check.hpp"

#include <cstdlib>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::string fixture(const std::string& file)
{
    return std::string(TEMPORAL_FIXTURE_DIR) + "/" + file;
}

ActionIdx index_of(const PlanningTask& task, const std::string& name)
{
    for (ActionIdx i = 0; i < task.actions.size(); ++i)
        if (task.actions[i].name == name) return i;
    throw std::runtime_error("no action named " + name);
}

// The state after running `name` in `s` and observing `event` (empty: the
// action's only outcome).
EpistemicState after(const PlanningTask& task, const EpistemicState& s, const std::string& name,
                     const std::string& event = "")
{
    const Action& a = task.actions[index_of(task, name)];
    auto outs = temporal::outcomes(task, s, a, make_world_cap_policy(task.partial_obs));
    if (outs.empty()) throw std::runtime_error(name + " is not applicable");
    if (event.empty()) {
        if (outs.size() != 1) throw std::runtime_error(name + " has several outcomes; name one");
        return outs.front().second;
    }
    for (auto& [e, state] : outs)
        if (a.events[e].name == event) return state;
    throw std::runtime_error(name + " cannot produce " + event + " here");
}

temporal::Commutation check(const PlanningTask& task, const EpistemicState& s,
                            const std::string& a, const std::string& b)
{
    return temporal::commutes(task, s, index_of(task, a), index_of(task, b));
}

// ── The corridor: one atom, two robots ──────────────────────────────────────

TEST(two_drives_commute)
{
    const auto task = load_task(fixture("robot-fleet.json"));
    const auto s0   = bisim_contract(task.init);

    const auto c = check(task, s0, "goto-junction_r1", "goto-junction_r2");
    CHECK_MSG(c.commute(), c.explain(task));
    CHECK(c.pairs == 1);
    // Both orders, one outcome each: two first updates and two second ones.
    CHECK(c.updates == 4);
}

TEST(an_action_does_not_overlap_itself)
{
    const auto task = load_task(fixture("robot-fleet.json"));
    const auto s0   = bisim_contract(task.init);

    // Driving to the junction twice: the second drive's precondition, not
    // being there, is what the first one ends.
    const auto c = check(task, s0, "goto-junction_r1", "goto-junction_r1");
    CHECK(c.verdict == temporal::Verdict::DisabledByFirst);
}

TEST(an_inapplicable_action_cannot_overlap_anything)
{
    const auto task = load_task(fixture("robot-fleet.json"));
    const auto s0   = bisim_contract(task.init);

    // r1 has not looked, so it cannot announce what it saw.
    const auto c = check(task, s0, "goto-junction_r2", "report-blocked_r1");
    CHECK(c.verdict == temporal::Verdict::Inapplicable);
    CHECK(task.actions[c.first].name == "report-blocked_r1");
}

TEST(two_looks_at_the_same_corridor_commute)
{
    const auto task = load_task(fixture("robot-fleet.json"));
    auto s = bisim_contract(task.init);
    s = after(task, s, "goto-junction_r1");
    s = after(task, s, "goto-junction_r2");

    // Both robots look at one corridor, each seeing that the other looked.
    // Whoever finishes first, both end up knowing, and knowing that the other
    // knows whether. Only the agreeing outcome pairs are possible: the
    // corridor is one way or the other.
    const auto c = check(task, s, "inspect_r1", "inspect_r2");
    CHECK_MSG(c.commute(), c.explain(task));
    CHECK(c.pairs == 2);
}

TEST(an_announcement_that_writes_nothing_still_disables_a_look)
{
    const auto task = load_task(fixture("robot-fleet.json"));
    auto s = bisim_contract(task.init);
    s = after(task, s, "goto-junction_r1");
    s = after(task, s, "goto-junction_r2");
    s = after(task, s, "inspect_r2", "e-inspect-blocked");

    // r2 has seen the corridor blocked; r1 has not looked. r2 announcing it and
    // r1 looking are both applicable, and neither writes an atom, so the atom
    // test calls them independent. They are not: the announcement makes r1
    // know, and looking requires that r1 does not yet know whether.
    const auto& report  = task.actions[index_of(task, "report-blocked_r2")];
    const auto& inspect = task.actions[index_of(task, "inspect_r1")];
    CHECK(temporal::syntactically_independent(report, inspect));

    const auto c = check(task, s, "report-blocked_r2", "inspect_r1");
    CHECK_MSG(!c.commute(), c.explain(task));
    CHECK_MSG(c.verdict == temporal::Verdict::DisabledByFirst, c.explain(task));
    CHECK(task.actions[c.first].name == "report-blocked_r2");
}

// ── The depot: two corridors, three robots ──────────────────────────────────

TEST(looks_at_different_corridors_commute_in_every_outcome)
{
    const auto task = load_task(fixture("robot-fleet-depot.json"));
    auto s = bisim_contract(task.init);
    s = after(task, s, "goto-north_r1");
    s = after(task, s, "goto-south_r2");

    // Four candidate worlds, both corridors open: every combination of the two
    // readings is possible, and each lands in the same model either way.
    const auto c = check(task, s, "inspect-north_r1", "inspect-south_r2");
    CHECK_MSG(c.commute(), c.explain(task));
    CHECK(c.pairs == 4);
}

// ── Coin in the box: who is watching ────────────────────────────────────────

TEST(who_was_watching_depends_on_the_order)
{
    const auto task = load_task(fixture("coin-in-the-box-multipointed.json"));
    const auto s0   = bisim_contract(task.init);

    // A opens the box, and A signals B to look. Both are applicable, and
    // either order leaves the box open and B looking. They still do not
    // commute: signalled first, B sees the opening; signalled second, B only
    // knows it is looking now, not what happened before it looked.
    const auto c = check(task, s0, "open_A", "signal_A_B");
    CHECK_MSG(c.verdict == temporal::Verdict::ResultsDiffer, c.explain(task));

    // Here the atom test agrees: the signal writes the atom the opening's
    // observability condition reads.
    CHECK(!temporal::syntactically_independent(task.actions[index_of(task, "open_A")],
                                               task.actions[index_of(task, "signal_A_B")]));
}

// ── Properties ──────────────────────────────────────────────────────────────

void check_symmetry(const PlanningTask& task, const EpistemicState& s, const std::string& where)
{
    for (ActionIdx a = 0; a < task.actions.size(); ++a) {
        for (ActionIdx b = a; b < task.actions.size(); ++b) {
            const auto ab = temporal::commutes(task, s, a, b);
            const auto ba = temporal::commutes(task, s, b, a);
            CHECK_MSG(ab.verdict == ba.verdict,
                      where + ": " + task.actions[a].name + " / " + task.actions[b].name +
                          ": " + temporal::verdict_name(ab.verdict) + " vs " +
                          temporal::verdict_name(ba.verdict));
        }
    }
}

TEST(the_verdict_does_not_depend_on_argument_order)
{
    const auto task = load_task(fixture("robot-fleet.json"));
    auto s = bisim_contract(task.init);
    check_symmetry(task, s, "initial");
    s = after(task, s, "goto-junction_r1");
    s = after(task, s, "goto-junction_r2");
    check_symmetry(task, s, "both at the junction");
    s = after(task, s, "inspect_r2", "e-inspect-blocked");
    check_symmetry(task, s, "r2 has seen it blocked");
}

TEST(commuting_actions_reach_the_same_state_by_either_path)
{
    // The definition, checked from the outside: for a commuting pair, running
    // the two in either order through the public API yields equal contracted
    // states for every outcome pair.
    const auto task = load_task(fixture("robot-fleet-depot.json"));
    auto s = bisim_contract(task.init);
    s = after(task, s, "goto-north_r1");
    s = after(task, s, "goto-south_r2");

    for (const std::string north : {"e-inspect-north-blocked", "e-inspect-north-clear"}) {
        for (const std::string south : {"e-inspect-south-blocked", "e-inspect-south-clear"}) {
            const auto one = after(task, after(task, s, "inspect-north_r1", north),
                                   "inspect-south_r2", south);
            const auto two = after(task, after(task, s, "inspect-south_r2", south),
                                   "inspect-north_r1", north);
            CHECK_MSG(one.fingerprint() == two.fingerprint(), north + " / " + south);
        }
    }
}

} // namespace

int main() { return run_all(); }
