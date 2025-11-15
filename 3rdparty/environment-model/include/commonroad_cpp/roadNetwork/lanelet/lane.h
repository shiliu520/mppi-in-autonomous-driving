#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "lanelet.h"

namespace geometry {
class CurvilinearCoordinateSystem;
}

using CurvilinearCoordinateSystem = geometry::CurvilinearCoordinateSystem;

/**
 * Class representing a lane.
 */
class Lane : public Lanelet {
  public:
    /**
     * Constructor initializing contained lanelet, lanelet describing lane, and Curvilinear coordinate system.
     *
     * @param containedLanelets Lanelets contained in lane.
     * @param lanelet Lanelet object spanning lane.
     * @param ccs Curvilinear coordinate system object.
     */
    Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet,
         std::shared_ptr<CurvilinearCoordinateSystem> ccs);

    /**
     * Constructor initializing contained lanelet, lanelet describing lane, and Curvilinear coordinate system.
     *
     * @param containedLanelets Lanelets contained in lane.
     * @param lanelet Lanelet object spanning lane.
     */
    Lane(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, Lanelet lanelet);

    Lane(const Lane &) = delete;
    Lane(Lane &&) = default;

    ~Lane();

    /**
     * Getter for curvilinear coordinate system using center line of lane as reference.
     *
     * @return Curvilinear coordinate system object.
     */
    [[nodiscard]] const std::shared_ptr<CurvilinearCoordinateSystem> &getCurvilinearCoordinateSystem();

    /**
     * Getter for lanelets contained in lane.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getContainedLanelets() const;

    /**
     * Getter for lanelet IDs contained in lane.
     *
     * @return Set of IDs of the lanelets contained in lane.
     */
    [[nodiscard]] const std::set<size_t> &getContainedLaneletIDs() const;

    /**
     * Collects all successor lanelets within lane given a start lanelet.
     *
     * @param lanelet Starting lanelet.
     * @return List of pointers to succeeding lanelets.
     */
    std::vector<std::shared_ptr<Lanelet>> getSuccessorLanelets(const std::shared_ptr<Lanelet> &lanelet) const;

    /**
     * Checks whether provided lanelet is part of lane.
     *
     * @param lanelet Pointer to lanelet object.
     * @return Boolean indicating whether lanelet is part of lane.
     */
    bool containsLanelet(const std::shared_ptr<Lanelet> &lanelet) const;

    /**
     * Checks whether lanelet with provided lanelet ID is part of lane.
     *
     * @param laneletId Lanelet ID.
     * @return Boolean indicating whether lanelet is part of lane.
     */
    bool containsLanelet(size_t laneletId) const;

    /**
     * Boolean indicating whether any provided lanelet is part of lane.
     *
     * @param lanelets List of pointers to lanelets.
     * @return Boolean indicating whether any lanelet is part of lane.
     */
    bool contains(std::vector<std::shared_ptr<Lanelet>> lanelets);

    /**
     * Checks whether provided lane is part of this lane.
     *
     * @param laneToCheck Lane to check if it is part of this lane
     * @return Boolean indicating whether lane is part of this lane
     */
    bool isPartOf(const std::shared_ptr<Lane> &laneToCheck);

  private:
    mutable std::vector<std::shared_ptr<Lanelet>>
        containedLanelets; //**< list of pointers to lanelets constructing lane */
    mutable std::shared_ptr<CurvilinearCoordinateSystem>
        curvilinearCoordinateSystem;      //**< curvilinear coordinate system defined by lane */
    std::set<size_t> containedLaneletIds; //**< set of IDs of the lanelets constructing lane */

    std::mutex ccs_lock;
};
