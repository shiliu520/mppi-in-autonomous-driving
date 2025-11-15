#pragma once

#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/planning_problem.h"
#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include <pugixml.hpp>
#include <stdexcept>

/**
 * Interface for reading a CommonRoad file.
 */
class CommonRoadFactory {
  public:
    /**
     * Virtual destructor (required for inheritance).
     */
    virtual ~CommonRoadFactory() = default;

    /**
     * Constructor for CommonRoadFactory.
     *
     * @param xmlDocument Pointer to loaded XML.
     */
    explicit CommonRoadFactory(std::unique_ptr<pugi::xml_document> xmlDocument) : doc(std::move(xmlDocument)) {}

    /**
     * Virtual function for creating obstacles.
     *
     * @return List of pointers to created obstacles.
     */
    virtual std::vector<std::shared_ptr<Obstacle>> createObstacles() = 0;

    /**
     * Virtual function for creating lanelets.
     *
     * @param sign List of pointers to traffic signs.
     * @param light List of pointers to traffic lights.
     * @return List of pointers to created lanelets.
     */
    virtual std::vector<std::shared_ptr<Lanelet>> createLanelets(std::vector<std::shared_ptr<TrafficSign>> sign,
                                                                 std::vector<std::shared_ptr<TrafficLight>> light) = 0;

    /**
     * Virtual function for creating traffic signs.
     *
     * @return List of pointers to created traffic signs.
     */
    virtual std::vector<std::shared_ptr<TrafficSign>> createTrafficSigns() = 0;

    /**
     * Virtual function for creating traffic lights.
     *
     * @return List of pointers to created traffic lights.
     */
    virtual std::vector<std::shared_ptr<TrafficLight>> createTrafficLights() = 0;

    /**
     * Virtual function for creating intersections.
     *
     * @param lanelets List of pointers to created lanelets.
     * @return List of pointers to created intersection.
     */
    virtual std::vector<std::shared_ptr<Intersection>>
    createIntersections(const std::vector<std::shared_ptr<Lanelet>> &lanelets) = 0;

    /**
     * Function for creating planning problems.
     *
     * @return List of pointers to created planning problems.
     */
    virtual std::vector<std::shared_ptr<PlanningProblem>> createPlanningProblems() = 0;

    /**
     * Virtual function for extracting the time step size.
     *
     * @return Time step size.
     */
    virtual double getTimeStepSize() = 0;

    /**
     * Extracts CommonRoad benchmark/scenario ID:
     *
     * @return Benchmark/scenario as string.
     */
    virtual std::string benchmarkID() = 0;

  protected:
    std::unique_ptr<pugi::xml_document> doc; //**< Pointer to loaded CommonRoad XML. */
};
