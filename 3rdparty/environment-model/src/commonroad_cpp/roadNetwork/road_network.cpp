#include <utility>

#include <boost/version.hpp>
#if (BOOST_VERSION / 100000) == 1 && (BOOST_VERSION / 100 % 1000) < 78 // Minor version < 78
// On Boost<1.78.0, the cartesian blanket header does not exist
#include <boost/geometry/strategies/strategies.hpp>

// On Boost<1.78.0, this needs to be included before <boost/geometry/index/parameters.hpp>
#include <boost/geometry/strategies/default_strategy.hpp>
#else
#include <boost/geometry/strategies/cartesian.hpp>
#endif

#include <boost/geometry/index/parameters.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

struct RoadNetwork::impl {
    bgi::rtree<value, bgi::quadratic<16>>
        rtree; //**< rtree defined by lanelets of road network for faster occupancy calculation*/
};

RoadNetwork::RoadNetwork(RoadNetwork &&) noexcept = default;

RoadNetwork::~RoadNetwork() = default;

RoadNetwork &RoadNetwork::operator=(RoadNetwork &&) noexcept = default;

RoadNetwork::RoadNetwork(const std::vector<std::shared_ptr<Lanelet>> &network, const SupportedTrafficSignCountry cou,
                         std::vector<std::shared_ptr<TrafficSign>> signs,
                         std::vector<std::shared_ptr<TrafficLight>> lights,
                         std::vector<std::shared_ptr<Intersection>> inters)
    : laneletNetwork(network), country(cou), trafficSigns(std::move(signs)), trafficLights(std::move(lights)),
      intersections(std::move(inters)), pImpl(std::make_unique<impl>()) {
    // construct Rtree out of lanelets
    for (const std::shared_ptr<Lanelet> &let : network)
        pImpl->rtree.insert(std::make_pair(let->getBoundingBox(), let->getId()));
    trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(cou);
}

const std::vector<std::shared_ptr<Lanelet>> &RoadNetwork::getLaneletNetwork() const { return laneletNetwork; }

const std::vector<std::shared_ptr<TrafficSign>> &RoadNetwork::getTrafficSigns() const { return trafficSigns; }

const std::vector<std::shared_ptr<TrafficLight>> &RoadNetwork::getTrafficLights() const { return trafficLights; }

std::vector<std::shared_ptr<Lane>> RoadNetwork::getLanes() const {
    std::vector<std::shared_ptr<Lane>> collectedLanes;
    for (const auto &[fst, snd] : lanes) {
        collectedLanes.push_back(snd.second);
    }
    return collectedLanes;
}

const std::vector<std::shared_ptr<Intersection>> &RoadNetwork::getIntersections() const { return intersections; }

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findOccupiedLaneletsByShape(const multi_polygon_type &polygonShape) {
    // find all relevant lanelets by making use of the rtree
    std::vector<value> relevantLanelets;
    for (const auto &polygon : polygonShape)
        pImpl->rtree.query(bgi::intersects(bg::return_envelope<box>(polygon.outer())),
                           std::back_inserter(relevantLanelets));
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    lanelets.reserve(relevantLanelets.size());
    for (auto [fst, snd] : relevantLanelets)
        lanelets.push_back(findLaneletById(snd));

    // check intersection with relevant lanelets
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets;
    for (const auto &let : lanelets) {
        for (const auto &polygon : polygonShape) {
            if (let->checkIntersection(polygon, ContainmentType::PARTIALLY_CONTAINED)) {
                occupiedLanelets.push_back(let);
                break;
            }
        }
    }
    return occupiedLanelets;
}

std::vector<std::shared_ptr<Lanelet>> RoadNetwork::findLaneletsByPosition(const double xPos, const double yPos) {
    std::vector<Lanelet> lanelet;
    polygon_type polygonPos;
    bg::append(polygonPos, point_type{xPos, yPos});

    return findOccupiedLaneletsByShape({polygonPos});
}

std::shared_ptr<Lanelet> RoadNetwork::findLaneletById(size_t laneletID) {
    const auto iter = std::find_if(std::begin(laneletNetwork), std::end(laneletNetwork),
                                   [laneletID](const auto &val) { return val->getId() == laneletID; });
    if (iter == std::end(laneletNetwork))
        throw std::domain_error("RoadNetwork::findLaneletById: Lanelet with ID " + std::to_string(laneletID) +
                                " does not exist in road network!");

    return *iter;
}

std::shared_ptr<TrafficLight> RoadNetwork::findTrafficLightById(size_t lightID) {
    const auto iter = std::find_if(std::begin(trafficLights), std::end(trafficLights),
                                   [lightID](const auto &val) { return val->getId() == lightID; });
    if (iter == std::end(trafficLights))
        throw std::domain_error("RoadNetwork::findTrafficLightById: Traffic light with ID " + std::to_string(lightID) +
                                " does not exist in road network!");

    return *iter;
}

SupportedTrafficSignCountry RoadNetwork::getCountry() const { return country; }

SupportedTrafficSignCountry RoadNetwork::matchStringToCountry(const std::string &name) {
    if (name == "DEU")
        return SupportedTrafficSignCountry::GERMANY;
    if (name == "USA")
        return SupportedTrafficSignCountry::USA;
    if (name == "ESP")
        return SupportedTrafficSignCountry::SPAIN;
    if (name == "ARG")
        return SupportedTrafficSignCountry::ARGENTINA;
    if (name == "BEL")
        return SupportedTrafficSignCountry::BELGIUM;
    if (name == "AUS")
        return SupportedTrafficSignCountry::AUSTRALIA;
    return SupportedTrafficSignCountry::ZAMUNDA;
}

std::string RoadNetwork::extractTrafficSignIDForCountry(const TrafficSignTypes type) const {
    return trafficSignIDLookupTable->at(type);
}

std::vector<std::shared_ptr<Lane>> RoadNetwork::addLanes(const std::vector<std::shared_ptr<Lane>> &newLanes,
                                                         size_t initialLanelet) {
    std::vector<std::shared_ptr<Lane>> updatedLanes;
    for (const auto &lane : newLanes) {
        if (lanes.count(lane->getContainedLaneletIDs()) != 0u and
            lanes[lane->getContainedLaneletIDs()].first.count(initialLanelet) != 0u) {
            updatedLanes.push_back(lanes.at(lane->getContainedLaneletIDs()).second);
        } else if (lanes.count(lane->getContainedLaneletIDs()) != 0u) {
            lanes[lane->getContainedLaneletIDs()].first.insert(initialLanelet);
            updatedLanes.push_back(lanes.at(lane->getContainedLaneletIDs()).second);
        } else {
            lanes[lane->getContainedLaneletIDs()] = {{initialLanelet}, lane};
            updatedLanes.push_back(lane);
        }
    }
    return updatedLanes;
}

std::vector<std::shared_ptr<Lane>> RoadNetwork::findLanesByBaseLanelet(const size_t laneletID) {
    std::vector<std::shared_ptr<Lane>> relevantLanes;
    for (const auto &[laneIDs, laneMap] : lanes)
        if (laneIDs.count(laneletID) != 0u and laneMap.first.count(laneletID) != 0u)
            relevantLanes.push_back(laneMap.second);
    return relevantLanes;
}

std::vector<std::shared_ptr<Lane>> RoadNetwork::findLanesByContainedLanelet(const size_t laneletID) {
    std::vector<std::shared_ptr<Lane>> relevantLanes;
    for (const auto &[laneIDs, laneMap] : lanes)
        if (laneIDs.count(laneletID) != 0u)
            relevantLanes.push_back(laneMap.second);
    return relevantLanes;
}
void RoadNetwork::setIdCounterRef(const std::shared_ptr<size_t> &idCounter) {
    if (idCounterRef == nullptr)
        idCounterRef = idCounter;
}

std::shared_ptr<size_t> RoadNetwork::getIdCounterRef() const { return idCounterRef; }

std::shared_ptr<IncomingGroup> RoadNetwork::findIncomingGroupByLanelet(const std::shared_ptr<Lanelet> &lanelet) const {
    for (const auto &inter : intersections)
        for (const auto &incom : inter->getIncomingGroups())
            for (const auto &let : incom->getIncomingLanelets())
                if (let->getId() == lanelet->getId())
                    return incom;
    return {};
}

std::shared_ptr<IncomingGroup>
RoadNetwork::findIncomingGroupByOutgoingGroup(const std::shared_ptr<OutgoingGroup> &outgoingGroup) const {
    for (const auto &inter : intersections)
        for (const auto &incom : inter->getIncomingGroups())
            if (incom->getOutgoingGroupID().has_value() and
                incom->getOutgoingGroupID().value() == outgoingGroup->getId())
                return incom;
    return {};
}

std::shared_ptr<OutgoingGroup> RoadNetwork::findOutgoingGroupByLanelet(const std::shared_ptr<Lanelet> &lanelet) const {
    for (const auto &inter : intersections)
        for (const auto &out : inter->getOutgoingGroups())
            for (const auto &let : out->getOutgoingLanelets()) {
                if (let->getId() == lanelet->getId())
                    return out;
            }
    return {};
}

const std::shared_ptr<LaneletGraph> &RoadNetwork::getTopologicalMap() const {
    if (topologicalMap != nullptr)
        return topologicalMap;
    topologicalMap = std::make_shared<LaneletGraph>(laneletNetwork);
    return topologicalMap;
}
