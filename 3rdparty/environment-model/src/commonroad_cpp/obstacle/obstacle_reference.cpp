#include <commonroad_cpp/obstacle/obstacle_reference.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>

std::vector<std::shared_ptr<Lane>>
obstacle_reference::computeRef(Obstacle &obstacle, const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {

    auto refLaneTmp{obstacle_reference::computeRefGivenLanes(
        obstacle, roadNetwork, timeStep, obstacle.getOccupiedLanesAndAdjacent(roadNetwork, timeStep))};

    if (refLaneTmp.empty()) {
        // try adjacent lanes
        std::vector<std::shared_ptr<Lanelet>> relevantLanelets;
        for (const auto &letBase : obstacle.getOccupiedLaneletsByShape(roadNetwork, timeStep))
            for (const auto &adj : lanelet_operations::adjacentLanelets(letBase, false))
                relevantLanelets.push_back(adj);
        auto lanes{lane_operations::createLanesBySingleLanelets(
            relevantLanelets, roadNetwork, obstacle.getSensorParameters().getFieldOfViewRear(),
            obstacle.getSensorParameters().getFieldOfViewFront(),
            obstacle.getRoadNetworkParameters().numIntersectionsPerDirectionLaneGeneration, {})};
        refLaneTmp = obstacle_reference::computeRefGivenLanes(obstacle, roadNetwork, timeStep, lanes);
    }
    if (refLaneTmp.empty() and obstacle.timeStepExists(timeStep - 1)) // use lanes from previous time step
        for (size_t prevTime{timeStep - 1}; timeStep >= obstacle.getFirstTimeStep(); --timeStep)
            refLaneTmp = obstacle_reference::computeRefGivenLanes(obstacle, roadNetwork, prevTime,
                                                                  obstacle.getOccupiedLanes(roadNetwork, prevTime));
    if (refLaneTmp.empty() and obstacle.timeStepExists(timeStep + 1)) // use next lanes from next time step
        for (size_t prevTime{timeStep + 1}; timeStep <= obstacle.getFinalTimeStep(); ++timeStep)
            refLaneTmp = obstacle_reference::computeRefGivenLanes(obstacle, roadNetwork, prevTime,
                                                                  obstacle.getOccupiedLanes(roadNetwork, prevTime));

    return refLaneTmp;
}

std::vector<std::shared_ptr<Lane>>
obstacle_reference::computeRefGivenLanes(Obstacle &obstacle, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                         size_t timeStep, const std::vector<std::shared_ptr<Lane>> &lanes) {
    std::vector<std::shared_ptr<Lane>> relevantOccupiedLanes;
    // use currently occupied lanelets in driving direction as lane candidates, already return if lane contains lanelet
    // of initial and final time step
    bool startEnd{false};
    bool startOrEnd{false};
    for (const auto &lane : lanes)
        if (lane->contains(
                obstacle.getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, obstacle.getFirstTimeStep())) and
            lane->contains(
                obstacle.getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, obstacle.getFinalTimeStep()))) {
            if (startEnd)
                relevantOccupiedLanes.push_back(lane);
            else {
                startEnd = true;
                relevantOccupiedLanes = {lane};
            }
            continue;
        } else if (!startEnd and lane->contains(obstacle.getOccupiedLaneletsDrivingDirectionByShape(
                                     roadNetwork, obstacle.getFinalTimeStep()))) {
            if (startOrEnd)
                relevantOccupiedLanes.push_back(lane);
            else {
                startOrEnd = true;
                relevantOccupiedLanes = {lane};
            }
            continue;
        } else if (!startEnd and !startOrEnd)
            relevantOccupiedLanes.push_back(lane);

    relevantOccupiedLanes = countOccupanciesOverTime(obstacle, roadNetwork, timeStep, relevantOccupiedLanes);
    return relevantOccupiedLanes;
}

std::vector<std::shared_ptr<Lane>>
obstacle_reference::countOccupanciesOverTime(Obstacle &obstacle, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                             size_t timeStep,
                                             std::vector<std::shared_ptr<Lane>> &relevantOccupiedLanes) {
    if (relevantOccupiedLanes.size() > 1) { // iterate over all time steps starting from current time step and count
                                            // occupied lanelets in driving direction which are part of lanes
        std::map<size_t, size_t> numOccupancies;
        for (size_t newTimeStep{timeStep}; newTimeStep <= obstacle.getFinalTimeStep(); ++newTimeStep) {
            auto occLanelets{obstacle.getOccupiedLaneletsRoadByShape(roadNetwork, newTimeStep)};
            for (const auto &lane : relevantOccupiedLanes)
                if (lane->contains(occLanelets))
                    numOccupancies[lane->getId()]++;
        }

        if (!numOccupancies.empty()) { // find lane with most occupancies
            auto mostOcc{
                std::max_element(numOccupancies.begin(), numOccupancies.end(),
                                 [](const auto &val1, const auto &val2) { return val1.second < val2.second; })};

            // add all lanes with the maximum number of occupancies to the result vector
            std::vector<std::shared_ptr<Lane>> maxOccupancyLanes;
            for (const auto &entry : numOccupancies) {
                if (entry.second == mostOcc->second) {
                    auto lane = std::find_if(
                        relevantOccupiedLanes.begin(), relevantOccupiedLanes.end(),
                        [&entry](const std::shared_ptr<Lane> &lane) { return lane->getId() == entry.first; });
                    auto idx{(*lane)->findClosestIndex(obstacle.getStateByTimeStep(timeStep)->getXPosition(),
                                                       obstacle.getStateByTimeStep(timeStep)->getYPosition())};
                    if (lane != relevantOccupiedLanes.end() and
                        (*lane)->getPathLength().back() - (*lane)->getPathLength().at(idx) >=
                            obstacle.getSensorParameters().getFieldOfViewFront() and
                        (*lane)->getPathLength().at(idx) - (*lane)->getPathLength().at(0) >=
                            obstacle.getSensorParameters().getFieldOfViewRear())
                        maxOccupancyLanes.push_back(*lane);
                }
            }
            // if check for fov is too restrictive, do not consider it (e.g., end of road network)
            if (maxOccupancyLanes.empty()) {
                for (const auto &entry : numOccupancies) {
                    if (entry.second == mostOcc->second) {
                        auto lane = std::find_if(
                            relevantOccupiedLanes.begin(), relevantOccupiedLanes.end(),
                            [&entry](const std::shared_ptr<Lane> &lane) { return lane->getId() == entry.first; });
                        if (lane != relevantOccupiedLanes.end())
                            maxOccupancyLanes.push_back(*lane);
                    }
                }
            }
            relevantOccupiedLanes = maxOccupancyLanes;

            // consider all lanes with highest fov coverage percentage
            if (relevantOccupiedLanes.size() > 1) {
                std::vector<std::shared_ptr<Lane>> lanesTmp;
                double highestFovCoverage{0.0};
                for (const auto &lane : relevantOccupiedLanes) {
                    auto idx{lane->findClosestIndex(obstacle.getStateByTimeStep(timeStep)->getXPosition(),
                                                    obstacle.getStateByTimeStep(timeStep)->getYPosition())};
                    auto fov{std::min(obstacle.getSensorParameters().getFieldOfViewFront() /
                                          (lane->getPathLength().back() - lane->getPathLength().at(idx)),
                                      1.0)};
                    auto rear{std::min(
                        obstacle.getSensorParameters().getFieldOfViewRear() / lane->getPathLength().at(idx), 1.0)};

                    if (fov + rear > highestFovCoverage) {
                        highestFovCoverage = lane->getPathLength().back();
                        lanesTmp = {lane};
                    }
                    if (fov + rear == highestFovCoverage) {
                        highestFovCoverage = lane->getPathLength().back();
                        lanesTmp.push_back(lane);
                    }
                }
                relevantOccupiedLanes = lanesTmp;
            }

            // check there exist several top num occupancies and if yes select the one which has the longest distance in
            // front
            if (relevantOccupiedLanes.size() > 1) {
                std::shared_ptr<Lane> laneLongestDistanceAhead;
                double longestDistanceAhead = std::numeric_limits<double>::max();
                for (const auto &lane : relevantOccupiedLanes) {
                    auto idx{lane->findClosestIndex(obstacle.getStateByTimeStep(timeStep)->getXPosition(),
                                                    obstacle.getStateByTimeStep(timeStep)->getYPosition())};
                    if (lane->getPathLength().back() - lane->getPathLength().at(idx) < longestDistanceAhead) {
                        longestDistanceAhead = lane->getPathLength().back();
                        laneLongestDistanceAhead = lane;
                    }
                }
                relevantOccupiedLanes = {laneLongestDistanceAhead};
            }
        }
    }
    return relevantOccupiedLanes;
}
