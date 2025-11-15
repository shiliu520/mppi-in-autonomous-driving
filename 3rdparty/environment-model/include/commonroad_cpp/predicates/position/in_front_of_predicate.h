#pragma once

#include "commonroad_cpp/predicates/commonroad_predicate.h"

/**
 * Evaluates whether the kth vehicle is in front of the pth vehicle.
 */
class InFrontOfPredicate : public CommonRoadPredicate {
  public:
    /**
     * Constructor for in front of predicate.
     */
    InFrontOfPredicate();

    /**
     * Boolean evaluation of predicate using parameter values.
     *
     * @param lonPositionK Longitudinal position of the kth obstacle [m].
     * @param lonPositionP Longitudinal position of the pth obstacle [m].
     * @param lengthK Length of the kth obstacle [m].
     * @param lengthP Length of the pth obstacle [m].
     * @return Boolean indicating satisfaction of the predicate.
     */
    static bool booleanEvaluation(double lonPositionP, double lonPositionK, double lengthP, double lengthK);

    /**
     * Boolean evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                           const std::shared_ptr<Obstacle> &obstacleP, const std::shared_ptr<Obstacle> &obstacleK,
                           const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                           bool setBased = false) override;

    /**
     * Constraint evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle.
     * @return Constraints defined by the predicate.
     */
    Constraint constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Obstacle> &obstacleP,
                                    const std::shared_ptr<Obstacle> &obstacleK,
                                    const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                                    bool setBased = false) override;

    /**
     * Constraint evaluation of predicate using parameter values.
     *
     * @param lonPositionP Longitudinal position of the pth obstacle [m].
     * @param lengthK Length of the kth obstacle [m].
     * @param lengthP Length of the pth obstacle [m].
     * @return Constraints defined by the predicate.
     */
    static Constraint constraintEvaluation(double lonPositionP, double lengthK, double lengthP);

    /**
     * Robustness evaluation of predicate using objects.
     *
     * @param timeStep Time step of interest.
     * @param world World object.
     * @param obstacleK The kth obstacle.
     * @param obstacleP The pth obstacle. This is an optional parameter.
     * @return Real value indicating robustness of the predicate.
     */
    double robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                            const std::shared_ptr<Obstacle> &obstacleP, const std::shared_ptr<Obstacle> &obstacleK,
                            const std::vector<std::string> &additionalFunctionParameters = {"0.0"},
                            bool setBased = false) override;

    /**
     * Robustness evaluation of predicate using parameter values.
     *
     * @param lonPositionK Longitudinal position of the kth obstacle [m].
     * @param lonPositionP Longitudinal position of the pth obstacle [m].
     * @param lengthK Length of the kth obstacle [m].
     * @param lengthP Length of the pth obstacle [m].
     * @return Real value indicating robustness of the predicate.
     */
    static double robustEvaluation(double lonPositionP, double lonPositionK, double lengthP, double lengthK);
};
