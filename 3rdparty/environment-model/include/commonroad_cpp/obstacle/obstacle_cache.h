#pragma once
#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/geometry/types.h>
#include <map>
#include <tsl/robin_map.h>

class State;
class Occupancy;
class Lanelet;
class Lane;
namespace geometry {
class CurvilinearCoordinateSystem;
}

/**
 * Class representing cached obstacle elements.
 */
struct ObstacleCache {
    template <typename Value> using time_step_map_t = tsl::robin_map<time_step_t, Value>;
    //** type of history/trajectory prediction maps for physical  states */
    using state_map_t = time_step_map_t<std::shared_ptr<State>>;
    //** type of prediction maps for occupancies */
    using occupancy_map_t = time_step_map_t<std::shared_ptr<Occupancy>>;

    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>>
        occupiedLanelets{}; //**< map of time steps to lanelets occupied by the obstacle shape */

    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>>
        occupiedLaneletsState{}; //**< map of time steps to lanelets occupied by the obstacle states (no shape
                                 // considered) */

    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>>
        occupiedLaneletsFront{}; //**< map of time steps to lanelets in driving direction occupied by the obstacles
                                 // front
                                 //*/

    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>>
        occupiedLaneletsBack{}; //**< map of time steps to lanelets in driving direction occupied by the obstacles back
                                //*/

    mutable time_step_map_t<std::vector<std::shared_ptr<Lane>>>
        occupiedLanesDrivingDir{}; //**< map of time steps to lanelets occupied by the obstacle */

    mutable time_step_map_t<std::vector<std::shared_ptr<Lanelet>>>
        occupiedLaneletsDrivingDir{}; //**< map of time steps to lanelets in driving direction occupied by the obstacle
                                      //*/

    mutable time_step_map_t<std::vector<std::shared_ptr<Lanelet>>>
        occupiedLaneletsNotDrivingDir{}; //**< map of time steps to lanelets in not driving direction occupied by the
                                         // obstacle */

    mutable time_step_map_t<std::shared_ptr<Lane>>
        referenceLane{}; //**< lane which is used as reference for curvilinear projection */

    mutable time_step_map_t<std::vector<std::shared_ptr<Lane>>>
        occupiedLanes{}; //**< map of time steps to lanes occupied by the obstacle */

    mutable time_step_map_t<std::vector<double>>
        frontXYPositions{}; //**< map of time steps to frontXY position of the obstacle */

    mutable time_step_map_t<std::vector<double>>
        backXYPositions{}; //**< map of time steps to backXY position of the obstacle */

    mutable time_step_map_t<double> leftLatPosition{}; //**< map of time step to left lat position */

    mutable time_step_map_t<double> rightLatPosition{}; //**< map of time step to right lat position */

    mutable time_step_map_t<std::map<size_t, double>>
        lateralDistanceToObjects{}; //**< map of time steps to map of other obstacles and the regarding distance to the
                                    // obstacle */

    using curvilinear_position_t = std::array<double, 3>; //**< curvilinear position of an obstacle */
    using curvilinear_position_map_t =
        tsl::robin_pg_map<std::shared_ptr<geometry::CurvilinearCoordinateSystem>,
                          curvilinear_position_t>; //**< map from CCS to curvilinear positions */
    mutable time_step_map_t<curvilinear_position_map_t>
        convertedPositions{}; //**< map of time steps to CCS to curvilinear positions */

    mutable time_step_map_t<multi_polygon_type> shapeAtTimeStep{}; //**< occupied polygon shape at time steps */

    /**
     * Resets helper mappings for specific obstacle time step
     *
     * @param timeStep Relevant time step.
     * @param clearReferenceLane Boolean indicating whether reference lane should be cleared.
     */
    void removeTimeStepFromMappingVariables(const size_t timeStep, const bool clearReferenceLane) {
        occupiedLanelets.erase(timeStep);
        occupiedLaneletsState.erase(timeStep);
        occupiedLaneletsFront.erase(timeStep);
        occupiedLaneletsBack.erase(timeStep);
        occupiedLanesDrivingDir.erase(timeStep);
        occupiedLaneletsDrivingDir.erase(timeStep);
        occupiedLaneletsNotDrivingDir.erase(timeStep);
        if (clearReferenceLane)
            referenceLane.erase(timeStep);
        occupiedLanes.erase(timeStep);
        frontXYPositions.erase(timeStep);
        backXYPositions.erase(timeStep);
        leftLatPosition.erase(timeStep);
        rightLatPosition.erase(timeStep);
        lateralDistanceToObjects.erase(timeStep);
        convertedPositions.erase(timeStep);
        shapeAtTimeStep.erase(timeStep);
    }

    /**
     * Clears all mappings.
     */
    void clear() {
        occupiedLanelets.clear();
        occupiedLaneletsState.clear();
        occupiedLaneletsFront.clear();
        occupiedLaneletsBack.clear();
        occupiedLanesDrivingDir.clear();
        occupiedLaneletsDrivingDir.clear();
        occupiedLaneletsNotDrivingDir.clear();
        referenceLane.clear();
        occupiedLanes.clear();
        frontXYPositions.clear();
        backXYPositions.clear();
        leftLatPosition.clear();
        rightLatPosition.clear();
        lateralDistanceToObjects.clear();
        convertedPositions.clear();
        shapeAtTimeStep.clear();
    }
};
