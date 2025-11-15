#include <algorithm>
#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <stdexcept>
#include <utility>

void TrafficLight::setId(const size_t num) { id = num; }

void TrafficLight::setCycle(const std::vector<TrafficLightCycleElement> &lightCycle) { cycle = lightCycle; }

void TrafficLight::setOffset(const size_t timeStepsOffset) { offset = timeStepsOffset; }

size_t TrafficLight::getId() const { return id; }

size_t TrafficLight::getOffset() const { return offset; }

std::vector<TrafficLightCycleElement> TrafficLight::getCycle() const { return cycle; }

TrafficLightCycleElement TrafficLight::getElementAtTime(const size_t time) const {
    std::vector cycleInitTimeSteps{offset};
    for (size_t i{0}; i < cycle.size(); ++i) {
        if (i == 0)
            cycleInitTimeSteps.push_back(cycle.at(i).duration + offset);
        else
            cycleInitTimeSteps.push_back(cycle.at(i).duration + offset + cycleInitTimeSteps.back());
    }

    size_t timeStepMod{(time - offset) % (cycleInitTimeSteps.back() - offset) + offset};

    const auto cycleIndex{std::distance(cycleInitTimeSteps.begin(),
                                        std::find_if(cycleInitTimeSteps.begin(), cycleInitTimeSteps.end(),
                                                     [timeStepMod](const size_t cyc) { return timeStepMod < cyc; })) -
                          1};
    return cycle.at(static_cast<unsigned long>(cycleIndex));
}

Direction TrafficLight::getDirection() const { return direction; }

void TrafficLight::setDirection(const Direction dir) { direction = dir; }

bool TrafficLight::isActive() const { return active; }

void TrafficLight::setActive(const bool trafficLightActive) { active = trafficLightActive; }

void TrafficLight::addCycleElement(const TrafficLightCycleElement cycleElement) { cycle.push_back(cycleElement); }

void TrafficLight::setPosition(const vertex pos) { position = pos; }

vertex TrafficLight::getPosition() const { return position; }

TrafficLightState TrafficLight::matchTrafficLightState(const std::string &trafficLightState) {
    std::string str{trafficLightState};
    std::transform(str.begin(), str.end(), str.begin(), toupper);
    str.erase(remove(str.begin(), str.end(), '_'), str.end());
    if (TrafficLightStateNames.count(str) == 1)
        return TrafficLightStateNames.at(str);
    throw std::logic_error("TrafficLight::matchTrafficLightState: Invalid traffic light state '" + str + "'!");
}

TrafficLight::TrafficLight(const size_t trafficLightId, std::vector<TrafficLightCycleElement> cycle,
                           const size_t offset, const Direction direction, const bool active, const vertex &position)
    : id(trafficLightId), cycle(std::move(cycle)), offset(offset), direction(direction), active(active),
      position(position) {}
