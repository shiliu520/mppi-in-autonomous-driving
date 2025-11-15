#pragma once

#include <commonroad_cpp/auxiliaryDefs/structs.h>
#include <cstddef>
#include <string>
#include <vector>

/**
 * Class representing a CommonRoad traffic light.
 */
class TrafficLight {
  public:
    /**
     * Default Constructor without parameters for a traffic light.
     */
    TrafficLight() = default;

    /**
     * Constructor of traffic light.
     *
     * @param trafficLightId Traffic light id.
     * @param cycle Traffic light cycle.
     * @param offset Traffic light offset.
     * @param direction Traffic light direction.
     * @param active Indicator whether traffic light is currently active.
     * @param position Traffic light position.
     */
    TrafficLight(size_t trafficLightId, std::vector<TrafficLightCycleElement> cycle, size_t offset, Direction direction,
                 bool active, const vertex &position);

    /**
     * Setter for traffic light ID.
     *
     * @param num Traffic light ID.
     */
    void setId(size_t num);

    /**
     * Setter for traffic light cycle.
     *
     * @param lightCycle List of traffic light cycle elements.
     */
    void setCycle(const std::vector<TrafficLightCycleElement> &lightCycle);

    /**
     * Setter for traffic light offset.
     *
     * @param timeStepsOffset Time offset.
     */
    void setOffset(size_t timeStepsOffset);

    /**
     * Setter for traffic light direction.
     *
     * @param dir Direction of the traffic light.
     */
    void setDirection(Direction dir);

    /**
     * Setter for traffic light activity.
     *
     * @param trafficLightActive Boolean indicating whether traffic light is active.
     */
    void setActive(bool trafficLightActive);

    /**
     * Setter for traffic light position.
     *
     * @param pos Position of traffic light.
     */
    void setPosition(vertex pos);

    /**
     * Adds a cycle element to the traffic light.
     *
     * @param cycleElement Traffic light cycle element.
     */
    void addCycleElement(TrafficLightCycleElement cycleElement);

    /**
     * Getter for traffic light ID.
     *
     * @return ID of traffic light.
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for traffic light cycle.
     *
     * @return List of traffic light cycle elements.
     */
    [[nodiscard]] std::vector<TrafficLightCycleElement> getCycle() const;

    /**
     * Getter for traffic light offset.
     *
     * @return Time offset of traffic light.
     */
    [[nodiscard]] size_t getOffset() const;

    /**
     * Getter for traffic light direction.
     *
     * @return Direction of traffic light.
     */
    [[nodiscard]] Direction getDirection() const;

    /**
     * Getter for traffic light activity indicator.
     *
     * @return Boolean indicating whether traffic light is active.
     */
    [[nodiscard]] bool isActive() const;

    /**
     * Getter for traffic light position.
     *
     * @return Position of traffic light.
     */
    [[nodiscard]] vertex getPosition() const;

    /**
     * Computes traffic light cycle element at a given time step.
     *
     * @param time Time step of interest.
     * @return Traffic light cycle element which is active at the provided time step.
     */
    [[nodiscard]] TrafficLightCycleElement getElementAtTime(size_t time) const;

    /**
     * Matches a traffic light state given as string to the corresponding enum value.
     *
     * @param trafficLightState String representing traffic light state.
     * @return Traffic light state enum value.
     */
    [[nodiscard]] static TrafficLightState matchTrafficLightState(const std::string &trafficLightState);

  private:
    size_t id;                                   //**< unique ID of traffic light */
    std::vector<TrafficLightCycleElement> cycle; //**< cycle of traffic light */
    size_t offset;                               //**< time offset of traffic light */
    Direction direction;                         //**< direction for which traffic light is valid */
    bool active;                                 //**< boolean indicating whether traffic light is valid */
    vertex position;                             //**< position of traffic light */
};
