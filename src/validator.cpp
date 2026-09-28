#include "validator.hpp"
#include "product_update.hpp"
#include "bisimulation.hpp"
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>

namespace {

// Plans are DAGs: a (node, state) pair already validated is not replayed.
struct VisitHash {
    std::size_t operator()(const std::pair<const PlanNode*, Fingerprint>& k) const noexcept {
        return FingerprintHash{}(k.second) ^ std::hash<const PlanNode*>{}(k.first);
    }
};
using Visited = std::unordered_set<std::pair<const PlanNode*, Fingerprint>, VisitHash>;

// A state in which some agent believes a contradiction, named for the error
// message, or nothing. Only asked when the task requires consistent beliefs.
std::optional<std::string> collapse(const EpistemicState& s, const PlanningTask& task) {
    if (task.seriality() != Seriality::Require) return std::nullopt;
    const auto c = find_collapse(s);
    if (!c) return std::nullopt;
    return "agent " + task.agent_names[c->agent] +
           " believes a contradiction at world " + std::to_string(c->world);
}

} // namespace

static void replay(const EpistemicState& s,
                   const std::shared_ptr<PlanNode>& node,
                   const PlanningTask& task,
                   ValidationResult& result,
                   Visited& visited) {

    // Checked here and not left to the update, so that the error names the
    // agent. Every state the plan reaches passes through this point, the
    // initial one included.
    if (auto why = collapse(s, task)) {
        result.valid = false;
        result.error = *why;
        return;
    }

    // Null node means this branch is at goal
    if (!node) {
        result.leaves_reached++;
        if (!s.satisfies(*task.goal)) {
            result.valid = false;
            std::ostringstream oss;
            oss << "Leaf reached but goal not satisfied ("
                << result.leaves_reached << " leaves so far)";
            result.error = oss.str();
        }
        return;
    }

    if (!visited.insert({node.get(), s.fingerprint()}).second) return;

    const auto it = task.action_index.find(node->action);
    const Action* action = it == task.action_index.end() ? nullptr : &task.actions[it->second];
    if (!action) {
        result.valid = false;
        result.error = "Action not found in task: " + node->action;
        return;
    }

    //   - ontic actions: ∀ designated worlds must satisfy precondition
    //   - sensing actions: ∃ (world, event) pair satisfies precondition
    // This ensures the validator rejects plans the planner should never have produced.
    if (!action->applicable(s)) {
        result.valid = false;
        result.error = "Action not applicable (conformant check failed): " + node->action;
        return;
    }

    // Split product update — one branch per designated event. Uses the task's
    // frame and no world cap: validity must not depend on search limits.
    const Seriality seriality = task.seriality() == Seriality::Require ? Seriality::Ignore
                                                                         : task.seriality();
    auto branches = product_update_split(s, *action, seriality, make_world_cap_policy(true));
    if (branches.empty()) {
        result.valid = false;
        result.error = "product_update_split returned empty for: " + node->action;
        return;
    }

    result.branches_checked++;

    // Match each plan branch to a product update branch by EventIdx
    for (auto& [plan_eid, subtree] : node->branches) {
        bool found = false;
        for (auto& [actual_eid, branch_state] : branches) {
            if (actual_eid != plan_eid) continue;
            found = true;
            EpistemicState contracted = bisim_contract(branch_state);
            replay(contracted, subtree, task, result, visited);
            if (!result.valid) return;
            break;
        }
        if (!found) {
            result.valid = false;
            std::ostringstream oss;
            oss << "Plan branch event " << plan_eid
                << " not produced by action " << node->action;
            result.error = oss.str();
            return;
        }
    }
}

ValidationResult validate(const PlanningTask& task,
                          const std::shared_ptr<PlanNode>& plan_tree) {
    ValidationResult result;
    result.valid = true;

    EpistemicState init = bisim_contract(task.init);

    if (!plan_tree) {
        // Empty plan — goal must hold in initial state
        result.leaves_reached = 1;
        if (auto why = collapse(init, task)) {
            result.valid = false;
            result.error = *why;
        } else if (!init.satisfies(*task.goal)) {
            result.valid = false;
            result.error = "Empty plan but initial state does not satisfy goal";
        }
        return result;
    }

    Visited visited;
    replay(init, plan_tree, task, result, visited);
    return result;
}