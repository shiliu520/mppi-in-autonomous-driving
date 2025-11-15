#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <tsl/robin_map.h>

#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "commonroad_cpp/obstacle/actuator_parameters.h"
#include "commonroad_cpp/obstacle/sensor_parameters.h"
#include "commonroad_cpp/obstacle/time_parameters.h"
#include "commonroad_cpp/roadNetwork/road_network_config.h"

class RoadNetwork;
class Obstacle;
class State;

/**
 * Storage for general parameters.
 */
class WorldParameters {
  public:
    WorldParameters() = default;

    /**
     * Constructor of world parameters.
     *
     * @param roadNetworkParameters Parameters for road network.
     * @param sensorParameters Parameters for obstacle sensors.
     * @param actuatorParametersEgo Actuator parameters for ego vehicle.
     * @param timeParameters Time parameters for obstacles.
     * @param actuatorParametersObstacles Actuator parameters for obstacles.
     */
    WorldParameters(const RoadNetworkParameters &roadNetworkParameters, const SensorParameters &sensorParameters,
                    const ActuatorParameters &actuatorParametersEgo, const TimeParameters &timeParameters,
                    const ActuatorParameters &actuatorParametersObstacles)
        : roadNetworkParams(roadNetworkParameters), sensorParams(sensorParameters),
          actuatorParamsEgo(actuatorParametersEgo), timeParams(timeParameters),
          actuatorParamsObstacles(actuatorParametersObstacles), defaultParams(false) {}

    /**
     * Getter for road network parameters.
     *
     * @return Road network parameters.
     */
    [[nodiscard]] RoadNetworkParameters getRoadNetworkParams() const { return roadNetworkParams; }

    /**
     * Getter for sensor parameters.
     *
     * @return Sensor parameters.
     */
    [[nodiscard]] SensorParameters getSensorParams() const { return sensorParams; }

    /**
     * Setter for time step size.
     * @param timeStepSize Time step size [s].
     */
    void setTimeStepSize(const double timeStepSize) { timeParams.setTimeStepSize(timeStepSize); }

    /**
     * Getter for actuator parameters.
     *
     * @return Actuator parameters.
     */
    [[nodiscard]] ActuatorParameters getActuatorParamsEgo() const { return actuatorParamsEgo; }

    /**
     * Getter for time parameters.
     *
     * @return Time parameters.
     */
    [[nodiscard]] TimeParameters getTimeParams() const { return timeParams; }

    /**
     * Getter for actuator parameters.
     *
     * @return Actuator parameters.
     */
    [[nodiscard]] ActuatorParameters getActuatorParamsObstacles() const { return actuatorParamsObstacles; }

    /**
     * Checks whether default parameters are set.
     *
     * @return Boolean indicating whether default parameters are set.
     */
    [[nodiscard]] bool hasDefaultParams() const { return defaultParams; }

  private:
    RoadNetworkParameters roadNetworkParams{RoadNetworkParameters()};        //**< Parameters for road network. */
    SensorParameters sensorParams{SensorParameters::dynamicDefaults()};      /** Parameters for obstacle sensors. */
    ActuatorParameters actuatorParamsEgo{ActuatorParameters::egoDefaults()}; /** Parameters for ego actuators. */
    TimeParameters timeParams{TimeParameters::dynamicDefaults()};            /** Parameters for obstacle time. */
    ActuatorParameters actuatorParamsObstacles{
        ActuatorParameters::vehicleDefaults()}; /** Parameters for obstacle actuators. */
    bool defaultParams{true};                   /** Boolean indicating whether default parameters are set. */
};

class World {
  public:
    /**
     * Constructor for world.
     *
     * @param name ID/Name of world.
     * @param timeStep Current time step of world object.
     * @param roadNetwork Currently relevant road network.
     * @param egos List of ego vehicles.
     * @param otherObstacles List of obstacles.
     * @param timeStepSize Time step size [s].
     * @param worldParams Parameters for world.
     */
    World(std::string name, size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
          std::vector<std::shared_ptr<Obstacle>> egos, std::vector<std::shared_ptr<Obstacle>> otherObstacles,
          double timeStepSize, const WorldParameters &worldParams = WorldParameters());

    /**
     * Default constructor without parameters for a scenario.
     */
    World() = default;

    /**
     * Getter for world time step.
     *
     * @return Time step of world.
     */
    [[nodiscard]] size_t getTimeStep() const;

    /**
     * Getter for pointer to road network object.
     *
     * @return Pointer to road network object.
     */
    [[nodiscard]] std::shared_ptr<RoadNetwork> getRoadNetwork() const;

    /**
     * Getter for pointer to vector of ego vehicle objects.
     *
     * @return Vector with pointers to obstacle objects.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Obstacle>> &getEgoVehicles() const;

    /**
     * Setter for ego vehicles of world
     *
     * @param egos vector of pointer to obstacles
     */
    void setEgoVehicles(const std::vector<std::shared_ptr<Obstacle>> &egos);

    /**
     * Sets ego vehicles for scenario by ids and moves the respective obstacles from obstacles to egoVehicles. Existing
     * ego vehicles are moved to obstacles.
     *
     * @param egos List of ego vehicle IDs
     */
    void setEgoVehicles(std::vector<size_t> &egos);

    /**
     * Getter for pointer to vector of ego vehicle objects.
     *
     * @return Vector with pointers to obstacle objects.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Obstacle>> &getObstacles() const;

    /**
     * Find all obstacles objects from given set of IDs.
     *
     * @return Vector with pointers to obstacle objects.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Obstacle>> findObstacles(const std::vector<size_t> &obstacleIdList) const;

    /**
     * Find all obstacles object corresponding to given ID.
     *
     * @return Obstacle object.
     */
    [[nodiscard]] std::shared_ptr<Obstacle> findObstacle(size_t obstacleId) const;

    /**
     * Computes all curvilinear states for ego vehicles and obstacles.
     */
    void setCurvilinearStates() const;

    /**
     * Creates pointer to ID counter so that it can be increased by other classes.
     * @return Pointer to ID counter.
     */
    [[nodiscard]] std::shared_ptr<size_t> getIdCounterRef() const;

    /**
     * Getter for time step size.
     *
     * @return Time step size [s].
     */
    [[nodiscard]] double getDt() const;

    /**
     * Getter for name.
     *
     * @return Name of world.
     */
    [[nodiscard]] const std::string &getName() const;

    /**
     * Updates and adds obstacles. Moves current state to history and updates prediction.
     * Adds new obstacles and remove non-existing obstacles if history is empty.
     *
     * @param obstacleList List of new and updated obstacles.
     */
    void updateObstacles(const std::vector<std::shared_ptr<Obstacle>> &obstacleList);

    /**
     * Updates and adds obstacles. Moves current state to history and updates prediction.
     * Adds new obstacles and remove non-existing obstacles if history is empty.
     * New obstacles must be provided via obstacleList. For obstacles which should be updated, current state,
     * trajectory, and shape must be provided separately. For providing new and already present obstacles as obstacle
     * list, use updateObstacles.
     *
     * @param obstacleList List of new obstacles.
     * @param currentStates Map of obstacle IDs to current states.
     * @param trajectoryPredictions Map of obstacle IDs to trajectory predictions.
     */
    void
    updateObstaclesTraj(const std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                        std::map<size_t, std::shared_ptr<State>> &currentStates,
                        std::map<size_t, tsl::robin_map<time_step_t, std::shared_ptr<State>>> &trajectoryPredictions);

    /**
     * Getter for world parameters.
     *
     * @return World parameters.
     */
    [[nodiscard]] WorldParameters getWorldParameters() const;

    /**
     * Propagate world time, i.e., current state is added to history and first prediction time step becomes current
     * state.
     *
     * @param ego Boolean indicating whether ego vehicles should be propagated.
     */
    void propagate(bool ego = true) const;

    /**
     * Resets the obstacle cache for all obstacles in the world.
     */
    void resetObstacleCache() const;

  private:
    std::string name;                                   //**< ID/name of world. */
    size_t timeStep;                                    //**< reference time step where world was created. */
    size_t idCounter{0};                                //**< counter to ensure unique IDs among all objects. */
    std::shared_ptr<RoadNetwork> roadNetwork;           //**< road network containing lanelets, traffic signs, etc. */
    std::vector<std::shared_ptr<Obstacle>> egoVehicles; //**< pointers to ego vehicle objects */
    std::vector<std::shared_ptr<Obstacle>> obstacles;   //**< pointers to obstacles *
    double dt;                                          //**<Time step size [s] *
    WorldParameters worldParameters; //**< General parameters for world, e.g. obstacles, road network. */

    /**
     * Initializes missing state information, e.g, acceleration or reaction time.
     */
    void initMissingInformation() const;

    /**
     * Computes for all ego vehicles occupied lanes per time step and sets reference lane. Also computes road lanes.
     */
    void setInitialLanes() const;
};
