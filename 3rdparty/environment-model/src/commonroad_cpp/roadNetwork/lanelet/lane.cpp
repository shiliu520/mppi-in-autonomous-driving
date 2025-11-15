#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/road_network_config.h>
#include <geometry/curvilinear_coordinate_system.h>
#include <utility>

#include <spdlog/spdlog.h>

/* NOTE: The Lane destructor is explicitly instantiated here so that the deallocation calls are hopefully
 * compiled with the same settings as the allocation calls.
 * This is critical since the CCS uses Eigen internally which uses a custom allocator to ensure appropriate
 * alignment of certain structures. If different allocators are used for the allocation and deallocation,
 * silent memory corruption will occur resulting in hard to find bugs.
 */
Lane::~Lane() {}

Lane::Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet,
           std::shared_ptr<CurvilinearCoordinateSystem> ccs)
    : Lanelet(std::move(lanelet)), containedLanelets(containedLanelets), curvilinearCoordinateSystem(std::move(ccs)) {
    for (const auto &coLa : containedLanelets)
        containedLaneletIds.insert(coLa->getId());
}

Lane::Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet)
    : Lanelet(std::move(lanelet)), containedLanelets(containedLanelets) {
    for (const auto &coLa : containedLanelets)
        containedLaneletIds.insert(coLa->getId());
}

const std::vector<std::shared_ptr<Lanelet>> &Lane::getContainedLanelets() const { return containedLanelets; }

const std::shared_ptr<CurvilinearCoordinateSystem> &Lane::getCurvilinearCoordinateSystem() {
    std::unique_lock lock{ccs_lock};

    if (!curvilinearCoordinateSystem) {
        geometry::EigenPolyline temp_path;
        geometry::EigenPolyline reference_path;
        const auto &centerVertices = getCenterVertices();
        temp_path.reserve(centerVertices.size());
        for (auto vert : centerVertices)
            temp_path.emplace_back(vert.x, vert.y);

        SPDLOG_DEBUG("Reference Path - initial size: {}", temp_path.size());
        geometry::util::chaikins_corner_cutting(temp_path, RoadNetworkParameters::cornerCuttingRefinements,
                                                reference_path);
        SPDLOG_DEBUG("Reference Path - after chaikins_corner_cutting: {} (refinements: {})", reference_path.size(),
                     refinements);

        geometry::util::resample_polyline(reference_path, RoadNetworkParameters::stepsToResamplePolyline, temp_path);

        SPDLOG_DEBUG("Reference Path - after resampling: {} (step size: {})", temp_path.size(), polyline_step_size);

        reference_path = temp_path;

        curvilinearCoordinateSystem = std::make_shared<CurvilinearCoordinateSystem>(
            reference_path, RoadNetworkParameters::projectionDomainLimit, RoadNetworkParameters::eps1,
            RoadNetworkParameters::eps2, "off", 2);
    }

    return curvilinearCoordinateSystem;
}

const std::set<size_t> &Lane::getContainedLaneletIDs() const { return containedLaneletIds; }

bool Lane::containsLanelet(const std::shared_ptr<Lanelet> &lanelet) const { return containsLanelet(lanelet->getId()); }

bool Lane::containsLanelet(size_t laneletId) const {
    return containedLaneletIds.find(laneletId) != containedLaneletIds.cend();
}

std::vector<std::shared_ptr<Lanelet>> Lane::getSuccessorLanelets(const std::shared_ptr<Lanelet> &lanelet) const {
    if (getContainedLaneletIDs().find(lanelet->getId()) == getContainedLaneletIDs().end())
        return {};
    std::vector<std::shared_ptr<Lanelet>> relevantLanelets;
    auto succs{lanelet->getSuccessors()};
    while (!succs.empty())
        for (auto it = succs.begin(); it != succs.end(); ++it) {
            const auto &suc = *it;
            if (getContainedLaneletIDs().find(suc->getId()) != getContainedLaneletIDs().end()) {
                relevantLanelets.push_back(suc);
                succs = relevantLanelets.back()->getSuccessors();
                break;
            } else {
                it = succs.erase(it);
                if (it == succs.end())
                    break;
            }
        }

    return relevantLanelets;
}

bool Lane::contains(std::vector<std::shared_ptr<Lanelet>> lanelets) {
    auto containedLaneletIdsTmp{containedLaneletIds};
    return std::any_of(lanelets.begin(), lanelets.end(), [containedLaneletIdsTmp](const std::shared_ptr<Lanelet> &let) {
        return containedLaneletIdsTmp.find(let->getId()) != containedLaneletIdsTmp.end();
    });
}

bool Lane::isPartOf(const std::shared_ptr<Lane> &laneToCheck) {
    return std::includes(containedLaneletIds.begin(), containedLaneletIds.end(),
                         laneToCheck->getContainedLaneletIDs().begin(), laneToCheck->getContainedLaneletIDs().end());
}
