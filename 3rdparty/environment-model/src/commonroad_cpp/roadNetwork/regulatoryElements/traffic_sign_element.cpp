#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_element.h>

#include <utility>

TrafficSignElement::TrafficSignElement(TrafficSignTypes trafficSignType, std::vector<std::string> values)
    : trafficSignType(trafficSignType), additionalValues(std::move(values)) {}

TrafficSignTypes TrafficSignElement::getTrafficSignType() const { return trafficSignType; }

void TrafficSignElement::setTrafficSignType(TrafficSignTypes trafficSigntype) { trafficSignType = trafficSigntype; }

std::vector<std::string> TrafficSignElement::getAdditionalValues() const { return additionalValues; }

void TrafficSignElement::addAdditionalValue(const std::string &value) { additionalValues.push_back(value); }

void TrafficSignElement::setAdditionalValues(const std::vector<std::string> &values) { additionalValues = values; }

std::string TrafficSignElement::convertGermanTrafficSignIdToString(TrafficSignTypes signType) {
    return TrafficSignIDGermany.at(signType);
}
