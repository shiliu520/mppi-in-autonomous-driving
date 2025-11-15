#pragma once

#include <map>
#include <memory>
#include <vector>

#include "state.h"

#include "commonroad_cpp/obstacle/obstacle_cache.h"
#include <boost/container_hash/hash.hpp>
#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/geometry/shape.h>
#include <commonroad_cpp/geometry/types.h>
#include <commonroad_cpp/roadNetwork/road_network_config.h>

#include "actuator_parameters.h"
#include "sensor_parameters.h"
#include "signal_state.h"
#include "state_meta_info.h"
#include "time_parameters.h"

#include <tsl/robin_map.h>

class ShapeGroup;
class Lanelet;
class Lane;
class RoadNetwork;
class Occupancy;
namespace geometry {
class CurvilinearCoordinateSystem;
}

template <typename Value> using time_step_map_t = tsl::robin_map<time_step_t, Value>;
//** type of history/trajectory prediction maps for physical states */
using state_map_t = time_step_map_t<std::shared_ptr<State>>;
//** type of prediction maps for occupancies */
using occupancy_map_t = time_step_map_t<std::shared_ptr<Occupancy>>;
//** type of history/trajectory prediction maps for signal states*/
using signal_state_map_t = time_step_map_t<std::shared_ptr<SignalState>>;

/**
 * Struct representing set-based prediction.
 */
struct SetBasedPrediction {
    occupancy_map_t setBasedPrediction{}; //**< set-based prediction of the obstacle */
    ObstacleCache obstacleCache{};        //**< cache for set-based prediction */

    /**
     * Resets helper mappings for obstacle time steps.
     *
     * @param timeStep Time step to remove from mapping variables.
     * @param clearReferenceLane Boolean indicating whether reference lane should be cleared.
     */
    void removeTimeStepFromMappingVariables(const size_t timeStep, const bool clearReferenceLane) {
        obstacleCache.removeTimeStepFromMappingVariables(timeStep, clearReferenceLane);
    }

    /**
     * Clears the cache for set-based prediction.
     */
    void clearCache() { obstacleCache.clear(); }
};

/**
 * Struct representing trajectory prediction.
 */
struct TrajectoryPrediction {
    signal_state_map_t signalSeries{};  //**< signal series of the obstacle */
    state_map_t trajectoryPrediction{}; //**< trajectory prediction of the obstacle */
    ObstacleCache obstacleCache{};      //**< cache for trajectory prediction */

    /**
     * Resets helper mappings for obstacle time steps.
     *
     * @param timeStep Time step to remove from mapping variables.
     * @param clearReferenceLane Boolean indicating whether reference lane should be cleared.
     */
    void removeTimeStepFromMappingVariables(const size_t timeStep, const bool clearReferenceLane) {
        obstacleCache.removeTimeStepFromMappingVariables(timeStep, clearReferenceLane);
    }

    /**
     * Clears the cache for trajectory prediction.
     */
    void clearCache() { obstacleCache.clear(); }
};

/**
 * Struct representing recorded states.
 */
struct RecordedStates {
    std::shared_ptr<State> currentState;             //**< pointer to current state of obstacle */
    std::shared_ptr<SignalState> currentSignalState; //**< pointer to current signal state of obstacle */
    state_map_t trajectoryHistory{};                 //**< previous states of the obstacle */
    signal_state_map_t signalSeriesHistory{};        //**< previous signal states of the obstacle */
    ObstacleCache occupancyRecorded;                 //**< cache for recorded occupancy (history + current time step) */

    /**
     * Resets helper mappings for obstacle time steps.
     *
     * @param timeStep Time step to remove from mapping variables.
     * @param clearReferenceLane Boolean indicating whether reference lane should be cleared.
     */
    void removeTimeStepFromMappingVariables(const size_t timeStep, const bool clearReferenceLane) {
        occupancyRecorded.removeTimeStepFromMappingVariables(timeStep, clearReferenceLane);
    }

    /**
     * Clears the cache for trajectory prediction.
     */
    void clearCache() { occupancyRecorded.clear(); }
};

/**
 * Class representing an obstacle.
 */
class Obstacle {
  public:
    /**
     * Default constructor without parameters for an obstacle.
     */
    Obstacle() = default;

    /**
     * Constructor initializing several obstacle attributes.
     * If the obstacle is static, certain values are overwritten.
     *
     * @param obstacleId ID of obstacle.
     * @param obstacleRole CommonRoad obstacle role, e.g, dynamic, static, environment, etc.
     * @param currentState Pointer to current state of obstacle.
     * @param obstacleType Type of the obstacle.
     * @param vMax Maximum velocity the obstacle can drive [m/s].
     * @param aMax Maximum acceleration the obstacle can have [m/s^2].
     * @param aMaxLong Maximum acceleration the obstacle can have in longitudinal direction [m/s^2].
     * @param aMinLong Minimum acceleration the obstacle can have in the longitudinal direction [m/s^2].
     * @param reactionTime Reaction time of the obstacle [s].
     * @param trajectoryPrediction Map matching time step to state.
     * @param length Length of the obstacle [m].
     * @param width Width of the obstacle [m].
     */
    Obstacle(size_t obstacleId, ObstacleRole obstacleRole, std::shared_ptr<State> currentState,
             ObstacleType obstacleType, double vMax, double aMax, double aMaxLong, double aMinLong, double reactionTime,
             state_map_t trajectoryPrediction, double length, double width);

    /**
     * Constructor initializing several obstacle attributes.
     * If the obstacle is static, certain values are overwritten.
     *
     * @param obstacleId ID of obstacle.
     * @param obstacleRole Boolean indicating whether the obstacle is static or not.
     * @param currentState Pointer to current state of obstacle.
     * @param obstacleType Type of the obstacle.
     * @param actuatorParameters Kinematic parameters of the obstacle.
     * @param sensorParameters Sensor parameters of the obstacle.
     * @param timeParameters Time parameters of the obstacle.
     * @param trajectoryPrediction Map matching time step to state.
     * @param shape Obstacle shape. (only rectangles are currently supported!)
     */
    Obstacle(size_t obstacleId, ObstacleRole obstacleRole, std::shared_ptr<State> currentState,
             ObstacleType obstacleType, const ActuatorParameters &actuatorParameters, SensorParameters sensorParameters,
             const TimeParameters &timeParameters, state_map_t trajectoryPrediction, std::unique_ptr<Shape> shape);

    /**
     * Setter for ID of obstacle.
     *
     * @param obstacleId ID of obstacle.
     */
    void setId(size_t obstacleId);

    /**
     * Setter for isStatic.
     *
     * @param staticObstacle Boolean indicating whether the obstacle is static or not.
     */
    void setIsStatic(bool staticObstacle);

    /**
     * Setter for obstacle role.
     *
     * @param type Role of obstacle
     */
    void setObstacleRole(ObstacleRole type);

    /**
     * Setter for current state. Overwrites existing state and resets related variables.
     *
     * @param currentState Current state of obstacle.
     */
    void setCurrentState(const std::shared_ptr<State> &currentState);

    /**
     * Updates current state. Moves current state to history.
     *
     * @param newState New current state of obstacle.
     */
    void updateCurrentState(const std::shared_ptr<State> &newState);

    /**
     * Setter for current signal state.
     *
     * @param state Current signal state of obstacle.
     */
    void setCurrentSignalState(const std::shared_ptr<SignalState> &state);

    /**
     * Setter for obstacle type.
     *
     * @param type Type of the obstacle.
     */
    void setObstacleType(ObstacleType type);

    /**
     * Setter for actuator parameters.
     *
     * @param actuatorParameters Actuator parameters
     */
    void setActuatorParameters(const ActuatorParameters &actuatorParameters);

    /**
     * Setter for time parameters.
     *
     * @param timeParameters Time parameters
     */
    void setTimeParameters(TimeParameters timeParameters);

    /**
     * Setter for sensor parameters.
     *
     * @param sensorParameters Sensor parameters
     */
    void setSensorParameters(SensorParameters sensorParameters);

    /**
     * Setter for road network parameters.
     *
     * @param roadNetworkParameters Road network parameters
     */
    void setRoadNetworkParameters(RoadNetworkParameters roadNetworkParameters);

    /**
     * Setter for trajectory prediction.
     *
     * @param trajPrediction Map matching time step to state.
     */
    void setTrajectoryPrediction(const state_map_t &trajPrediction);

    /**
     * Setter for trajectory history.
     *
     * @param trajHistory Map matching time step to state.
     */
    void setTrajectoryHistory(const state_map_t &trajHistory);

    /**
     * Setter for obstacle shape
     * @param shape Shape
     */
    void setGeoShape(std::unique_ptr<Shape> shape);

    /**
     * Appends a state to the trajectory prediction.
     *
     * @param state Pointer to state object.
     */
    void appendStateToTrajectoryPrediction(const std::shared_ptr<State> &state);

    /**
     * Appends an occupancy to the set based prediction.
     *
     * @param occ Occupancy object.
     */
    void appendOccupancyToSetBasedPrediction(const std::shared_ptr<Occupancy> &occ);

    /**
     * Appends a state to the history.
     *
     * @param state Pointer to state object.
     */
    void appendStateToHistory(const std::shared_ptr<State> &state);

    /**
     * Appends a signal state to the signal series.
     *
     * @param state Pointer to signal state object.
     */
    void appendSignalStateToSeries(const std::shared_ptr<SignalState> &state);

    /**
     * Appends a signal state to the history.
     *
     * @param state Pointer to signal state object.
     */
    void appendSignalStateToHistory(const std::shared_ptr<SignalState> &state);

    /**
     * Getter for obstacle ID.
     *
     * @return Obstacle ID.
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for obstacle role.
     *
     * @return Role of obstacle
     */
    [[nodiscard]] ObstacleRole getObstacleRole() const;

    /**
     * Getter for current state.
     *
     * @return Pointer to state object.
     */
    [[nodiscard]] const std::shared_ptr<State> &getCurrentState() const;

    /**
     * Getter current signal state.
     *
     * @return Pointer to signal state object.
     */
    [[nodiscard]] const std::shared_ptr<SignalState> &getCurrentSignalState() const;

    /**
     * Getter for obstacle type.
     *
     * @return Type of the obstacle.
     */
    [[nodiscard]] ObstacleType getObstacleType() const;

    /**
     * Getter for actuator parameters.
     *
     * @return Actuator parameters
     */
    ActuatorParameters getActuatorParameters() const;

    /**
     * Getter for sensor parameters.
     *
     * @return Sensor parameters
     */
    SensorParameters getSensorParameters() const;

    /**
     * Getter for time parameters.
     *
     * @return Time parameters
     */
    TimeParameters getTimeParameters() const;

    /**
     * Getter for road network parameters.
     *
     * @return Road network parameters
     */
    RoadNetworkParameters getRoadNetworkParameters() const;

    /**
     * Getter for maximum velocity the vehicle can drive.
     *
     * @return Maximum velocity [m/s].
     */
    [[nodiscard]] double getVmax() const;

    /**
     * Getter for maximum acceleration.
     *
     * @return Maximum acceleration [m/s^2].
     */
    [[nodiscard]] double getAmax() const;

    /**
     * Getter for maximum acceleration in longitudinal direction.
     *
     * @return Maximum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAmaxLong() const;

    /**
     * Getter for minimum acceleration in longitudinal direction.
     *
     * @return Minimum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAminLong() const;

    /**
     * Getter for reaction time.
     *
     * @return Reaction time [s].
     */
    [[nodiscard]] double getReactionTime() const;

    /**
     * Getter for reference lane.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork Pointer to road network
     * @return Pointer to lane object.
     */
    [[nodiscard]] std::shared_ptr<Lane> getReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                         time_step_t timeStep);

    /**
     * Getter for trajectory prediction.
     *
     * @return Map matching time step to state.
     */
    [[nodiscard]] state_map_t getTrajectoryPrediction() const;

    /**
     * Getter for trajectory history.
     *
     * @return Map matching time step to state.
     */
    [[nodiscard]] state_map_t getTrajectoryHistory() const;

    /**
     * Getter for polygon shape of obstacle at given time step.
     *
     * @param timeStep Time step of interest.
     * @return Boost polygon.
     */
    [[nodiscard]] multi_polygon_type getOccupancyPolygonShape(time_step_t timeStep);

    /**
     * Getter for obstacle shape.
     *
     * @return Shape object.
     */
    [[nodiscard]] Shape &getGeoShape() const;

    /**
     * Getter for occupied lanelets at a time steps within a road network.
     *
     * @param roadNetwork Road network object.
     * @param timeStep Time step of interest
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of pointers to occupied lanelets.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>>
    getOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep,
                               bool setBased = false);

    /**
     * Extracts occupied lanes and adjacent lanes of them for given time step.
     *
     * @param roadNetwork CommonRoad road network.
     * @param timeStep Time step of interest.
     * @return List of pointers to lanes.
     */
    std::vector<std::shared_ptr<Lane>> getOccupiedLanesAndAdjacent(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                   size_t timeStep);

    /**
     * Extracts the occupied lanelets in driving direction for given time step. Lanelets in opposite driving direction
     * are not considered.
     *
     * @param roadNetwork CommonRoad road network.
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of occupied lanelets.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>>
    getOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep,
                                               bool setBased = false);

    /**
     * Extracts the occupied lanelets of road for given time step. Lanelets in opposite driving direction are
     * considered.
     *
     * @param roadNetwork CommonRoad road network.
     * @param timeStep Time step of interest.
     * @return List of pointers to occupied lanelets.
     */
    std::vector<std::shared_ptr<Lanelet>>
    getOccupiedLaneletsRoadByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

    /**
     * Extracts the occupied lanes in driving direction for given time step. Lanes in opposite driving direction are not
     * considered.
     *
     * @param roadNetwork CommonRoad road network.
     * @param timeStep Time step of interest.
     * @return List of pointers to occupied lanes.
     */
    std::vector<std::shared_ptr<Lane>> getOccupiedLanesDrivingDirection(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                        size_t timeStep);

    /**
     * Sets the occupied lanelets in driving direction for given time step. Lanes in opposite driving direction are not
     * considered.
     *
     * @param roadNetwork CommonRoad road network.
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of pointers to occupied lanelets.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>>
    setOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep,
                                               bool setBased = false);

    /**
     * Sets the occupied lanelets not in driving direction for given time step.
     *
     * @param roadNetwork CommonRoad road network.
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of pointers to occupied lanelets.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>>
    getOccupiedLaneletsNotDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep,
                                                  bool setBased);

    /**
     * Getter for occupied lanelets by the front of the car at a time steps within a road network.
     *
     * @param roadNetwork Road network object.
     * @param timeStep Time step of interest
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of pointers to occupied lanelets by front of the car.
     */
    std::vector<std::shared_ptr<Lanelet>> getOccupiedLaneletsByFront(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                     size_t timeStep, bool setBased = false);

    /**
     * Getter for occupied lanelets by the back of the car at a time steps within a road network.
     *
     * @param roadNetwork Road network object.
     * @param timeStep Time step of interest
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of pointers to occupied lanelets by back of the car.
     */
    std::vector<std::shared_ptr<Lanelet>> getOccupiedLaneletsByBack(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                    size_t timeStep, bool setBased = false);

    /**
     * Provides state given a time step. The time step can belong to the current state, history, or prediction.
     *
     * @param timeStep Time step of interest.
     * @return Pointer to state object.
     */
    [[nodiscard]] std::shared_ptr<State> getStateByTimeStep(time_step_t timeStep) const;

    /**
     * Provides signalState given a time step. The time step can belong to the current state, history, or prediction.
     *
     * @param timeStep Time step of interest.
     * @return Pointer to signalState object.
     */
    [[nodiscard]] std::shared_ptr<SignalState> getSignalStateByTimeStep(time_step_t timeStep) const;

    /**
     * Returns the length of the trajectory prediction.
     *
     * @return Length of trajectory prediction.
     */
    [[nodiscard]] size_t getTrajectoryLength() const;

    /**
     * Computes the global coordinates of the front of the car.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return Vector of double with X- and Y-Coordinate-Point.
     */
    std::vector<double> getFrontXYCoordinates(time_step_t timeStep, bool setBased = false);

    /**
     * Computes the global coordinates of the back of the car.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return Vector of double with X- and Y-Coordinate-Point.
     */
    std::vector<double> getBackXYCoordinates(time_step_t timeStep, bool setBased = false);

    /**
     * Computes the maximum longitudinal front position of obstacle (for rectangle shapes)
     *
     * @param timeStep time step of interest
     * @param roadNetwork Pointer to road network
     * @return longitudinal position of obstacle front
     */
    double frontS(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Computes the maximum longitudinal front position of obstacle (for rectangle shapes) based on a given reference
     * curvilinear coordinate system.
     *
     * @param timeStep Time step of interest.
     * @param ccs Pointer to reference CCS which should be used.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return Longitudinal position of obstacle front
     */
    double frontS(time_step_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                  bool setBased = false);

    /**
     * Computes the minimum longitudinal rear position of obstacle (for rectangle shapes)
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return longitudinal position of obstacle rear
     */
    double rearS(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Computes the minimum longitudinal rear position of obstacle (for rectangle shapes) based on given reference
     * curvilinear coordinate system.
     *
     * @param timeStep Time step of interest.
     * @param ccs Pointer to reference CCS which should be used.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return Longitudinal position.
     */
    double rearS(time_step_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                 bool setBased = false);

    /**
     * Calculates right d-coordinate of vehicle
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step to consider
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return right d-coordinate [m]
     */
    double rightD(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep, bool setBased = false);

    /**
     * Calculates right lateral position of obstacle based on given reference curvilinear coordinate system
     *
     * @param timeStep time step to consider
     * @param ccs Pointer to reference CCS which should be used.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return lateral position [m]
     */
    double rightD(time_step_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                  bool setBased = false) const;

    /**
     * Calculates left d-coordinate of vehicle
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step to consider
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return left d-coordinate [m]
     */
    double leftD(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep, bool setBased = false);

    /**
     * Calculates left lateral position of obstacle based on given reference curvilinear coordinate system
     *
     * @param timeStep time step to consider
     * @param ccs Pointer to reference CCS which should be used.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return lateral position [m]
     */
    double leftD(time_step_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                 bool setBased = false) const;

    /**
     * Computes the longitudinal position of obstacle based on Cartesian state and assigned reference lane
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return longitudinal position of obstacle state
     */
    [[nodiscard]] double getLonPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Computes the longitudinal position of obstacle based on Cartesian state and provided reference CCS.
     *
     * @param timeStep Time Step of interest.
     * @param ccs Pointer to reference CCS which should be used.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return longitudinal position of obstacle state
     */
    [[nodiscard]] double getLonPosition(time_step_t timeStep,
                                        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                        bool setBased = false) const;

    /**
     * Computes the lateral position of obstacle based on Cartesian state and assigned reference lane
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return lateral position of obstacle state
     */
    [[nodiscard]] double getLatPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Computes the lateral position of obstacle based on Cartesian state and assigned reference CCS
     *
     * @param timeStep Time step of interest.
     * @param ccs Pointer to reference CCS which should be used.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return lateral position of obstacle state
     */
    [[nodiscard]] double getLatPosition(time_step_t timeStep,
                                        const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                        bool setBased = false) const;

    /**
     * Computes the curvilinear orientation of obstacle based on Cartesian state and assigned lane
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep time step of interest
     * @return curvilinear orientation of obstacle state
     */
    [[nodiscard]] double getCurvilinearOrientation(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                   time_step_t timeStep); // Todo create test case

    /**
     * Computes the curvilinear orientation of obstacle based on Cartesian state and provided reference
     * curvilinear coordinate system.
     *
     * @param timeStep Time step of interest.
     * @param ccs Pointer to reference CCS which should be used.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return curvilinear orientation of obstacle state
     */
    [[nodiscard]] double getCurvilinearOrientation(time_step_t timeStep,
                                                   const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                                   bool setBased = false);

    /**
     * Sets the lanes from the road network the obstacle occupies at a certain time step
     *
     * @param lanes List of existing lanes.
     * @param timeStep time step of interest
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     */
    void setOccupiedLanes(const std::vector<std::shared_ptr<Lane>> &lanes, time_step_t timeStep,
                          bool setBased = false); // TODO create test case

    /**
     * Computes occupied lanes at a time step.
     *
     * @param roadNetwork Pointer to road network.
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     */
    void setOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep,
                          bool setBased = false); // TODO create test case

    /**
     * Extracts first time step of trajectory
     *
     * @return first time step of trajectory
     */
    [[nodiscard]] size_t getFirstTrajectoryTimeStep() const; // TODO create test case

    /**
     * Getter for occupied lanes at a time step. Computes occupied lanes if not happened already.
     *
     * @param roadNetwork Pointer to road network.
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of pointers to occupied lanes.
     */
    std::vector<std::shared_ptr<Lane>> getOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                        time_step_t timeStep,
                                                        bool setBased = false); // TODO create test case

    /**
     * Extracts the occupied lanes of road for given time step. Lanes in opposite driving direction are considered.
     * @param roadNetwork CommonRoad road network.
     * @param timeStep Time step of interest.
     * @return List of pointers to occupied lanes.
     */
    std::vector<std::shared_ptr<Lane>> getOccupiedRoadLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                            size_t timeStep); // TODO create test case

    /**
     * Converts the x- and y-coordinate into the Curvilinear domain given own reference lane.
     *
     * @param roadNetwork Pointer to road network
     * @param timeStep Time step for which the coordinates should be transformed.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     */
    void convertPointToCurvilinear(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep,
                                   bool setBased = false);

    /**
     * Checks whether time steps exists in trajectory prediction or current state.
     *
     * @param timeStep Time step of interest.
     * @return Boolean indicating whether time step exists.
     */
    bool timeStepExists(time_step_t timeStep) const;

    /**
     * Interpolates acceleration based on velocity.
     *
     * @param timeStep Time step for which acceleration should be interpolated.
     * @param timeStepSize Time step size [s].
     */
    void interpolateAcceleration(time_step_t timeStep, double timeStepSize) const;

    /**
     * Getter for all prediction time steps.
     *
     * @return List of time steps of prediction.
     */
    std::vector<size_t> getPredictionTimeSteps() const;

    /**
     * Getter for all history time steps.
     *
     * @return List of time steps of prediction.
     */
    std::vector<size_t> getHistoryTimeSteps() const;

    /**
     * Getter for list of time steps containing current time step and prediction time steps.
     *
     * @return List of time steps.
     */
    std::vector<size_t> getTimeSteps() const;

    /**
     * Computes occupied lanes for each time step of obstacle and sets reference lane.
     *
     * @param roadNetwork Pointer to road network.
     * @param considerHistory Boolean indicating whether history should be considered for computation
     */
    void computeLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, bool considerHistory = false);

    /**
     * Converts all states to curvilinear representation.
     *
     * @param roadNetwork Pointer to road network
     */
    void setCurvilinearStates(const std::shared_ptr<RoadNetwork> &roadNetwork);

    /**
     * Converts position at a given time step to curvilinear coordinate system given a reference CCS.
     * Point is stored locally in variable convertedPositions.
     *
     * @param timeStep time step of interest
     * @param ccs Reference curvilinear coordinate system (CCS) which should be used.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     */
    void convertPointToCurvilinear(size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                   bool setBased = false) const;

    /**
     * Getter for field of view area.
     *
     * @return Field of view area as polygon.
     */
    const polygon_type getFov();

    /**
     * Setter for field of view area.
     *
     * @param fovVertices Vertices representing polygon.
     */
    void setFov(const std::vector<vertex> &fovVertices);

    /**
     * Getter for signal series.
     *
     * @return Map representing signal series.
     */
    const signal_state_map_t &getSignalSeries() const;

    /**
     * Getter for signal series history.
     *
     * @return Map representing signal series history.
     */
    const signal_state_map_t &getSignalSeriesHistory() const;

    /**
     * Checks whether obstacle is static.
     *
     * @return Boolean indicating whether obstacle is static.
     */
    bool isStatic() const;

    /**
     * Computes driven distance of obstacle starting at current state.
     *
     * @return Driven distance [m].
     */
    double drivenTrajectoryDistance() const;

    /**
     * Returns distance to obstacle
     * @param timeStep time step of interest
     * @param obs Obstacle of interest
     * @param roadnetwork Road network.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return Distance [m]
     */
    double getLateralDistanceToObstacle(time_step_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                        const std::shared_ptr<RoadNetwork> &roadnetwork, bool setBased = false);

    /**
     * Returns trajectory as vector (by default it is a map).
     *
     * @return List of time steps.
     */
    std::vector<std::shared_ptr<State>> trajectoryAsVector() const;

    /**
     * Getter for first time step of obstacle.
     *
     * @return First time step.
     */
    size_t getFirstTimeStep() const;

    /**
     * Getter for last time step of obstacle.
     *
     * @return Final time step.
     */
    size_t getFinalTimeStep() const;

    /**
     * Getter for lanelets which are occupied by state (not part of shape) by obstacle.
     *
     * @param roadNetwork Road network object.
     * @param timeStep Relevant time step.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of lanelets.
     */
    std::vector<std::shared_ptr<Lanelet>> getOccupiedLaneletsByState(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                     size_t timeStep, bool setBased = false);

    /**
     * Checks whether history contains relevant time steps.
     * @param currentTimeStep Current time step. Not equal to time step of currentState as obstacle might not be updated
     * for several time steps. Relevant for deleting obstacle from world.
     * @return Boolean indicating whether obstacle is still relevant for simulation.
     */
    bool historyPassed(size_t currentTimeStep) const;

    /**
     * Propagates time, i.e., current state is added to history and first prediction time step becomes current state.
     */
    void propagate();

    /**
     * Getter for set-based prediction.
     *
     * @return Set-based prediction of the obstacle.
     */
    occupancy_map_t getSetBasedPrediction() const;

    /**
     * Getter for velocity at a specific time step.
     * Function can consider set-based predictions and returns corresponding possible min/max velocity.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @param min Boolean indicating whether minimum or maximum velocity should be returned.
     *
     * @return Velocity [m/s].
     */
    double getVelocity(size_t timeStep, bool setBased = false, bool min = true) const;

    /**
     * Getter for acceleration at a specific time step.
     * Function can consider set-based predictions and returns corresponding possible min/max acceleration.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @param min Boolean indicating whether minimum or maximum acceleration should be returned.
     *
     * @return acceleration [m/s^2].
     */
    double getAcceleration(size_t timeStep, bool setBased = false, bool min = true) const;

    /**
     * Getter for shape as pointer.
     *
     * @return Shape as pointer.
     */
    std::unique_ptr<Shape> getShapePtr() const;

    /**
     * Manually sets reference lane. Internal variable for dynamic lane computation is set to false.
     *
     * @param refLane New reference lane.
     */
    void setReferenceLane(const std::shared_ptr<Lane> &refLane);

    /**
     * Clears the caches of predictions and recorded states.
     */
    void clearCache();

  private:
    size_t obstacleId;                                //**< unique ID of obstacle */
    ObstacleRole obstacleRole{ObstacleRole::DYNAMIC}; //**< CommonRoad obstacle role */
    ObstacleType obstacleType{ObstacleType::unknown}; //**< CommonRoad obstacle type */
    size_t firstTimeStep;                             //**< first time step (current state or in history */
    size_t finalTimeStep;                             //**< final time step (current state or in prediction */
    bool dynamicRef{true};                            //**< base reference is set manually */

    ActuatorParameters actuatorParameters{
        ActuatorParameters::vehicleDefaults()}; //**< actuator parameters, e.g., maximum velocity */
    SensorParameters sensorParameters{
        SensorParameters::dynamicDefaults()}; //**< sensor parameters, e.g., field of view */
    RoadNetworkParameters
        roadNetworkParameters; //**< road network parameters, e.g., required to create reference lane */
    TimeParameters timeParameters{TimeParameters::dynamicDefaults()};

    StateMetaInfo stateMetaInfo{};   //**< meta information relevant for obstacle */
    std::unique_ptr<Shape> geoShape; //**< shape of the obstacle */

    RecordedStates recordedStates{}; //**< recorded states of the obstacle consisting of history and current state*/
    TrajectoryPrediction trajectoryPrediction{}; //**< predicted states of the obstacle */
    SetBasedPrediction setBasedPrediction{};     //**< set-based prediction of the obstacle */

    /**
     * Private setter for occupied lanelets at a time steps within a road network.
     * Used to define critical section around it.
     *
     * @param roadNetwork Road network object.
     * @param timeStep Time step of interest
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of pointers to occupied lanelets.
     */
    std::vector<std::shared_ptr<Lanelet>> setOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                     time_step_t timeStep, bool setBased = false);

    /**
     * Sets the occupied lanelets not in driving direction for a time step.
     *
     * @param roadNetwork Road network.
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return List of occupied lanelets in driving direction.
     */
    std::vector<std::shared_ptr<Lanelet>>
    setOccupiedLaneletsNotDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep,
                                                  bool setBased = false);

    /**
     * Private setter for polygon shape of obstacle at given time step.
     * Used to define critical section around it.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered. Default is false.
     * @return Boost polygon.
     */
    multi_polygon_type setOccupancyPolygonShape(time_step_t timeStep, bool setBased = false);

    /**
     * Private setter for reference lane.
     * Used to define critical section around it.
     *
     * @param timeStep Time step of interest.
     * @param roadNetwork Pointer to road network
     * @return Pointer to lane object.
     */
    std::shared_ptr<Lane> setReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork, time_step_t timeStep);

    /**
     * Creates logging message in case of ccs conversion errors.
     *
     * @param timeStep Time step at which error occurred.
     * @param ccs Curvilinear coordinate system.
     * @param func Function where error occurred.
     * @return Error message.
     */
    std::string ccsErrorMsg(size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                            const std::string &func) const;

    /**
     * Extracts first and last time step of obstacle.
     */
    void setFirstLastTimeStep();

    /**
     * Updates history of obstacle.
     */
    void updateHistory();

    /**
     * Getter for occupied lanes cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of lanes per time step.
     */
    time_step_map_t<std::vector<std::shared_ptr<Lane>>> &getOccupiedLanesCache(size_t timeStep, bool setBased) const;

    /**
     * Getter for lateral distance to other obstacles cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of distance per time step.
     */
    time_step_map_t<std::map<size_t, double>> &getLateralDistanceToObjectCache(size_t timeStep, bool setBased) const;

    /**
     * Getter for rear position cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of position per time step.
     */
    time_step_map_t<std::vector<double>> &getBackXYCoordinatesCache(time_step_t timeStep, bool setBased) const;

    /**
     * Getter for front position cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of position per time step.
     */
    time_step_map_t<std::vector<double>> &getFrontXYCoordinatesCache(time_step_t timeStep, bool setBased) const;

    /**
     * Getter for occupied lanelets not in driving direction cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of occupied lanelets per time step.
     */
    time_step_map_t<std::vector<std::shared_ptr<Lanelet>>> &getOccupiedLaneletsNotDrivingDirCache(size_t timeStep,
                                                                                                  bool setBased) const;

    /**
     * Getter for occupied lanelets in driving direction cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of occupied lanelets per time step.
     */
    time_step_map_t<std::vector<std::shared_ptr<Lanelet>>> &getOccupiedLaneletsDrivingDirCache(size_t timeStep,
                                                                                               bool setBased) const;

    /**
     * Getter for converted curvilienar state position cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of converted curvilinear position per time step.
     */
    time_step_map_t<ObstacleCache::curvilinear_position_map_t> &convertedPositionsCache(size_t timeStep,
                                                                                        bool setBased) const;

    /**
     * Getter for reference lane cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of reference lane per time step.
     */
    time_step_map_t<std::shared_ptr<Lane>> &getReferenceLaneCache(size_t timeStep, bool setBased) const;

    /**
     * Getter for rigtht lateral position cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of right lateral position per time step.
     */
    time_step_map_t<double> &getRightLatPositionCache(size_t timeStep, bool setBased) const;

    /**
     * Getter for left lateral position cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of left lateral position per time step.
     */
    time_step_map_t<double> &getLeftLatPositionCache(size_t timeStep, bool setBased) const;

    /**
     * Getter for occupied lanelets cache (rear position).
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of occupied lanelets per time step.
     */
    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> &
    getOccupiedLaneletsBackCache(size_t timeStep, bool setBased);

    /**
     * Getter for occupied lanelets cache (front position).
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of occupied lanelets per time step.
     */
    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> &
    getOccupiedLaneletsFrontCache(size_t timeStep, bool setBased);

    /**
     * Getter for occupied lanelets cache (state occupancy).
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of occupied lanelets per time step.
     */
    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> &
    getOccupiedLaneletsStateCache(size_t timeStep, bool setBased);

    /**
     * Getter for occupied lanelets cache (shape occupancy).
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Map of occupied lanelets per time step.
     */
    std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> &getOccupiedLaneletsCache(size_t timeStep,
                                                                                                     bool setBased);

    /**
     * Getter for occupancy polygon shape cache.
     *
     * @param timeStep Time step of interest.
     * @param setBased Boolean indicating whether set-based prediction should be considered.
     * @return Mao of occupancy polygon shape as boost multi-polygon per time step.
     */
    time_step_map_t<multi_polygon_type> &getOccupancyPolygonShapeCache(size_t timeStep, bool setBased) const;
};
