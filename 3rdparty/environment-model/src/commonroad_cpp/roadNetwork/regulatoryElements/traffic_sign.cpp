#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>

#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include <algorithm>
#include <stdexcept>
#include <utility>

void TrafficSign::setId(const size_t num) { id = num; }

void TrafficSign::addTrafficSignElement(const std::shared_ptr<TrafficSignElement> &elem) {
    trafficSignElement.push_back(elem);
}

size_t TrafficSign::getId() const { return id; }

void TrafficSign::setTrafficSignElements(
    const std::vector<std::shared_ptr<TrafficSignElement>> &newTrafficSignElement) {
    trafficSignElement = newTrafficSignElement;
}

bool TrafficSign::isVirtualElement() const { return virtualElement; }

void TrafficSign::setVirtualElement(bool elem) { virtualElement = elem; }

std::vector<std::shared_ptr<TrafficSignElement>> TrafficSign::getTrafficSignElements() const {
    return trafficSignElement;
}

void TrafficSign::setPosition(vertex pos) { position = pos; }

vertex TrafficSign::getPosition() const { return position; }

TrafficSign::TrafficSign(size_t trafficSignId, std::vector<std::shared_ptr<TrafficSignElement>> traffic_sign_element,
                         const vertex &position, bool virtual_element)
    : id(trafficSignId), trafficSignElement(std::move(traffic_sign_element)), position(position),
      virtualElement(virtual_element) {}

std::vector<std::shared_ptr<TrafficSignElement>>
TrafficSign::getTrafficSignElementsOfType(const TrafficSignTypes &signType) const {
    std::vector<std::shared_ptr<TrafficSignElement>> relevantTrafficSignElements;
    for (const auto &signElement : trafficSignElement) {
        if (signElement->getTrafficSignType() == signType)
            relevantTrafficSignElements.push_back(signElement);
    }
    return relevantTrafficSignElements;
}

TrafficSignTypes TrafficSign::matchTrafficSign(const std::string &trafficSignName) {
    std::string str{trafficSignName};
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    if (TrafficSignNames.count(str) == 1)
        return TrafficSignNames.at(str);
    else
        throw std::logic_error("TrafficSign::matchTrafficSign: Invalid traffic sign name '" + str + "'!");
}
