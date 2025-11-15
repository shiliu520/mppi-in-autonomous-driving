#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/predicates/braking/causes_braking_intersection_predicate.h>

bool CausesBrakingIntersectionPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<Obstacle> &obstacleK,
                                                           const std::shared_ptr<Obstacle> &obstacleP,
                                                           const std::vector<std::string> &additionalFunctionParameters,
                                                           bool setBased) {
    if (obstacleP->getAcceleration(timeStep, setBased, true) >= parameters.getParam("aBrakingIntersection"))
        return false;
    std::vector<std::shared_ptr<Lane>> lanesP;
    std::vector<std::shared_ptr<Lane>> lanesK;

    // get all relevant lanes
    if (setBased and !obstacleP->getSetBasedPrediction().empty() and
        obstacleP->getCurrentState()->getTimeStep() < timeStep) {
        lanesP = obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep, setBased);
        lanesK = {obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)};
    } else if (setBased and !obstacleK->getSetBasedPrediction().empty() and
               obstacleK->getCurrentState()->getTimeStep() < timeStep) {
        lanesK = obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep, setBased);
        lanesP = {obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};
    } else {
        lanesK = {obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep)};
        lanesP = {obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};
    }

    // iterate over all lane pairs and compute intersection points; if between any intersection points and any obstacle
    // front position is below threshold return true
    for (const auto &laneP : lanesP) {
        std::vector<vertex> points;
        for (const auto &laneK : lanesK) {
            auto intersectionPoints{laneP->computeIntersectionPointsWithShape({laneK->getOuterPolygon()})};
            if (intersectionPoints.empty())
                continue;
            for (const auto &point : intersectionPoints) {
                double distance;
                auto pointCCSLon{
                    laneP->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(point.x, point.y).x()};
                if (setBased and !obstacleP->getSetBasedPrediction().empty() and
                    obstacleP->getCurrentState()->getTimeStep() < timeStep) {
                    for (const auto &obsShape : obstacleP->getOccupancyPolygonShape(timeStep))
                        for (const auto obsPoint : obsShape.outer()) {
                            polygon_type polygonPos;
                            boost::geometry::append(polygonPos, point_type{obsPoint.x(), obsPoint.y()});
                            if (!laneP->applyIntersectionTesting(polygonPos))
                                continue;
                            auto pointCCSLonObs{laneP->getCurvilinearCoordinateSystem()
                                                    ->convertToCurvilinearCoords(obsPoint.x(), obsPoint.y())
                                                    .x()};
                            distance = std::min(distance, pointCCSLon - pointCCSLonObs);
                        }
                } else
                    distance = pointCCSLon - obstacleP->frontS(timeStep, laneP->getCurvilinearCoordinateSystem());
                if (parameters.getParam("dCauseBrakingIntersection") <= distance and
                    distance <= parameters.getParam("dBrakingIntersection"))
                    return true;
            }
        }
    }
    return false;
}

Constraint CausesBrakingIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("CausesBrakingIntersectionPredicate does not support robust evaluation!");
}

double CausesBrakingIntersectionPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("CausesBrakingIntersectionPredicate does not support robust evaluation!");
}

CausesBrakingIntersectionPredicate::CausesBrakingIntersectionPredicate() : CommonRoadPredicate(true) {}
