#ifndef METRONOME_OFFLINEPLANMANAGER_HPP
#define METRONOME_OFFLINEPLANMANAGER_HPP

#include "PlanManager.hpp"
#include "util/TimeMeasurement.hpp"
namespace metronome {
template <typename Domain, typename Planner>
class OfflinePlanManager : PlanManager<Domain, Planner> {
public:
    Result plan(const Configuration& configuration, const Domain& domain, Planner& planner) {
        std::vector<typename Domain::Action> actions;
            long long int start = currentNanoTime();
        logTime();
        auto planningTime = measureNanoTime([&] { actions = planner.plan(domain.getStartState()); });
        long long int end = currentNanoTime();
        LOG(DEBUG) << (end - start) / 1000000;
            
        auto pathLength = actions.size();

        std::vector<std::string> actionStrings;

        for (auto& action : actions) {
            actionStrings.emplace_back(action.toString());
        }

        return Result(configuration,
                planner.getExpandedNodeCount(),
                planner.getGeneratedNodeCount(),
                planningTime,
                pathLength * configuration.getLong("actionDuration"),
                planningTime + pathLength * configuration.getLong("actionDuration"),
                planningTime,
                pathLength,
                actionStrings);
    }
};
}

#endif // METRONOME_OFFLINEPLANMANAGER_HPP
