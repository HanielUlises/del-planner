#include "strategy.hpp"

#include "hdelta.hpp"
#include "knowledge_relaxation.hpp"

std::unique_ptr<Heuristic> make_heuristic(const std::string& label,
                                          const PlanningTask& task) {
    if (label == "ug")   return std::make_unique<UnsatisfiedGoalHeuristic>();
    if (label == "ed")   return std::make_unique<EpistemicDistanceHeuristic>();
    if (label == "ks")   return std::make_unique<KnowledgeSpreadHeuristic>();
    if (label == "wc")   return std::make_unique<WorldCountHeuristic>();
    if (label == "rpg")  return std::make_unique<RelaxedClosureHeuristic>(RelaxedAggregation::Max);
    if (label == "radd") return std::make_unique<RelaxedClosureHeuristic>(RelaxedAggregation::Add);
    if (label == "kadd") return std::make_unique<KnowledgeRelaxationHeuristic>(task);
    if (label == "kff")  return std::make_unique<KnowledgeRelaxationHeuristic>(
                             task, KnowledgeRelaxationHeuristic::Estimate::FF);
    if (label == "hd")    return std::make_unique<hdelta::DistinguishabilityHeuristic>(false);
    if (label == "hdsum") return std::make_unique<hdelta::DistinguishabilityHeuristic>(true);
    return nullptr;
}

const char* heuristic_display(const std::string& label) {
    if (label == "ug")   return "unsatisfied-goal";
    if (label == "ed")   return "epistemic-distance";
    if (label == "ks")   return "knowledge-spread";
    if (label == "wc")   return "world-count";
    if (label == "rpg")  return "relaxed-closure (max)";
    if (label == "radd") return "relaxed-closure (add)";
    if (label == "kadd") return "knowledge-relaxation (add)";
    if (label == "kff")  return "knowledge-relaxation (ff)";
    if (label == "hd")    return "distinguishability (admissible)";
    if (label == "hdsum") return "distinguishability (sum)";
    return "unknown";
}

std::optional<Strategy> parse_strategy(const std::string& label) {
    if (label == "gbfs")   return Strategy::GBFS;
    if (label == "ehc")    return Strategy::EHC;
    if (label == "aostar") return Strategy::AOSTAR;
    if (label == "replan") return Strategy::REPLAN;
    if (label == "portfolio") return Strategy::PORTFOLIO;
    return std::nullopt;
}

const char* strategy_name(Strategy s) {
    switch (s) {
        case Strategy::AOSTAR: return "AO*";
        case Strategy::REPLAN: return "replan";
        case Strategy::PORTFOLIO: return "portfolio";
        case Strategy::EHC:    return "EHC";
        default:               return "GBFS";
    }
}

bool has_sensing_actions(const PlanningTask& task) {
    for (auto& action : task.actions)
        if (action.designated_events.size() > 1)
            return true;
    return false;
}
