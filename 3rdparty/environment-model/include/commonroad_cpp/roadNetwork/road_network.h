#pragma once

#include "commonroad_cpp/roadNetwork/intersection/outgoing_group.h"
#include "lanelet/lanelet_graph.h"
#include <boost/container_hash/hash.hpp>
#include <cstddef>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/geometry/types.h>

#include <commonroad_cpp/roadNetwork/types.h>

struct TrafficLightCycleElement;
using value = std::pair<box, unsigned>;

class Lanelet;
class Lane;
class IncomingGroup;
class TrafficLight;
class TrafficSign;
class Intersection;

/**
 * Class representing a road network.
 */
class RoadNetwork {
  public:
    /**
     * Constructor for RoadNetwork. Takes a lanelet network and automatically generates lanes out of them.
     * Additionally, creates and Rtree from the lanelets for faster access of lanelets and faster
     * occupancy computations of obstacles.
     *
     * @param network List of pointers to lanelets.
     * @param cou Country where road network is located.
     * @param signs List of traffic signs.
     * @param lights List of traffic lights.
     * @param inters List of intersections.
     */
    explicit RoadNetwork(
        const std::vector<std::shared_ptr<Lanelet>> &network,
        SupportedTrafficSignCountry cou = SupportedTrafficSignCountry::ZAMUNDA,
        std::vector<std::shared_ptr<TrafficSign>> signs = std::vector<std::shared_ptr<TrafficSign>>{},
        std::vector<std::shared_ptr<TrafficLight>> lights = std::vector<std::shared_ptr<TrafficLight>>{},
        std::vector<std::shared_ptr<Intersection>> inters = std::vector<std::shared_ptr<Intersection>>{});

    /**
     * Move assignment of road network.
     */
    RoadNetwork(RoadNetwork &&) noexcept;

    /*
     * Destructor of road network.
     */
    ~RoadNetwork();

    /**
     * Move constructor of road network.
     *
     * @return Road network.
     */
    RoadNetwork &operator=(RoadNetwork &&) noexcept;

    /**
     * Copy constructor of road network.
     */
    RoadNetwork(const RoadNetwork &) = delete;

    /**
     * Copy assignment of road network.
     *
     * @return Road network.
     */
    RoadNetwork &operator=(const RoadNetwork &) = delete;

    /**
     * Getter for lanelet network.
     *
     * @return list of pointers to lanelets
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLaneletNetwork() const;

    /**
     * Getter for traffic signs.
     *
     * @return list of traffic signs
     */
    [[nodiscard]] const std::vector<std::shared_ptr<TrafficSign>> &getTrafficSigns() const;

    /**
     * Getter for traffic lights.
     *
     * @return list of traffic lights
     */
    [[nodiscard]] const std::vector<std::shared_ptr<TrafficLight>> &getTrafficLights() const;

    /**
     * Getter for lanes.
     *
     * @return list of pointers to lanes
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lane>> getLanes() const;

    /**
     * Getter for intersections.
     *
     * @return list of pointers to intersections.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Intersection>> &getIntersections() const;

    /**
     * Getter for country.
     *
     * @return Country where road network is located.
     */
    [[nodiscard]] SupportedTrafficSignCountry getCountry() const;

    /**
     * Given a polygon shape, finds the list of lanelets within the road network which intersect with the shape.
     *
     * @param polygonShape boost polygon
     * @return list of lanelet pointers
     */
    std::vector<std::shared_ptr<Lanelet>> findOccupiedLaneletsByShape(const multi_polygon_type &polygonShape);

    /**
     * Given a position, finds the list of lanelets within the road network which contain the point.
     *
     * @param xPos x-coordinate of point
     * @param yPos y-coordinate of point
     * @return list of lanelet pointers
     */
    std::vector<std::shared_ptr<Lanelet>> findLaneletsByPosition(double xPos, double yPos);

    /**
     * Returns the lanelet which corresponds to a given lanelet ID.
     *
     * @param laneletID lanelet ID
     * @return pointer to lanelet
     */
    std::shared_ptr<Lanelet> findLaneletById(size_t laneletID);

    /**
     * Returns the traffic light which corresponds to a given traffic light ID.
     *
     * @param lightID traffic light ID
     * @return pointer to lanelet
     */
    std::shared_ptr<TrafficLight> findTrafficLightById(size_t lightID);

    /**
     * Matches a string to country enum.
     *
     * @param name Name of country.
     * @return Country enum.
     */
    static SupportedTrafficSignCountry matchStringToCountry(const std::string &name);

    /**
     * Extracts traffic sign ID string from type object
     *
     * @param type Traffic sign type
     * @return Traffic sign ID string.
     */
    std::string extractTrafficSignIDForCountry(TrafficSignTypes type) const;

    /**
     * Adds lanes to road network. It it is checked whether lane already exists.
     *
     * @param newLanes Pointers to lanes which should be added.
     * @param initialLanelet Lanelet based on which the lanes were created.
     * @return List of lanes which were added or already existed.
     */
    std::vector<std::shared_ptr<Lane>> addLanes(const std::vector<std::shared_ptr<Lane>> &newLanes,
                                                size_t initialLanelet);

    /**
     * Searches for existing lanes which were created based on lanelet with provided ID.
     *
     * @param laneletID Lanelet ID to search for.
     * @return Pointers to existing lanes.
     */
    std::vector<std::shared_ptr<Lane>> findLanesByBaseLanelet(size_t laneletID);

    /**
     * Searches for existing lanes which contain lanelet with provided ID.
     *
     * @param laneletID Lanelet ID to search for.
     * @return Pointers to existing lanes.
     */
    std::vector<std::shared_ptr<Lane>> findLanesByContainedLanelet(size_t laneletID);

    /**
     * Setter for idCounterRef.
     *
     * @param idCounter Pointer to ID counter of world object
     */
    void setIdCounterRef(const std::shared_ptr<size_t> &idCounter);

    /**
     * Getter for idCounterRef.
     *
     * @return Pointer to ID counter of world object
     */
    std::shared_ptr<size_t> getIdCounterRef() const;

    /**
     * Finds incoming object to which lanelet belongs. Returns empty pointer if lanelet is part of incoming.
     *
     * @param lanelet Lanelet of interest.
     * @return Incoming object
     */
    std::shared_ptr<IncomingGroup> findIncomingGroupByLanelet(const std::shared_ptr<Lanelet> &lanelet) const;

    /**
     * Finds incoming object to which outgoingGroup belongs. Returns empty pointer if outgoingGroup is part of incoming.
     *
     * @param outgoingGroup outgoingGroup of interest.
     * @return IncomingGroup object
     */
    std::shared_ptr<IncomingGroup>
    findIncomingGroupByOutgoingGroup(const std::shared_ptr<OutgoingGroup> &outgoingGroup) const;

    /**
     * Finds outgoingGroup to which lanelet belongs. Returns empty pointer if lanelet is part of outgoingGroup.
     *
     * @param lanelet Lanelet of interest.
     * @return OutgoingGroup object
     */
    std::shared_ptr<OutgoingGroup> findOutgoingGroupByLanelet(const std::shared_ptr<Lanelet> &lanelet) const;

    /**
     * Getter for topological map of road network.
     *
     * @return Lanelet graph.
     */
    const std::shared_ptr<LaneletGraph> &getTopologicalMap() const;

  private:
    std::vector<std::shared_ptr<Lanelet>> laneletNetwork;     //**< set of lanelets contained in road network */
    SupportedTrafficSignCountry country;                      //**< country where road network is located */
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns;   //**< set of traffic signs contained in road network */
    std::vector<std::shared_ptr<TrafficLight>> trafficLights; //**< set of traffic lights contained in road network */
    std::vector<std::shared_ptr<Intersection>> intersections; //**< set of intersections contained in road network */
    mutable std::shared_ptr<LaneletGraph> topologicalMap;

    std::unordered_map<lanelet_id_set, std::pair<lanelet_id_set, std::shared_ptr<Lane>>, boost::hash<lanelet_id_set>>
        lanes;
    //**< map of lanes contained in road network with the following structure: contained lanelet IDs, <base lanelets
    // used for creation of lane, lane object> */
    const std::unordered_map<TrafficSignTypes, std::string> *trafficSignIDLookupTable; //**< mapping of traffic signs*/
    std::shared_ptr<size_t> idCounterRef; //**< Pointer to ID counter of world object */

    //**< Struct for private fields including R-Tree */
    struct impl;
    std::unique_ptr<impl> pImpl;
};

// Required for lanes unordered_map in RoadNetwork class (unordered_set used as key)
namespace std {
template <typename V, typename H, typename P, typename A>
std::size_t hash_value(std::unordered_set<V, H, P, A> const &val) {
    return boost::hash_range(val.begin(), val.end());
}
} // namespace std
