#pragma once

#include <cstddef>

struct RoadNetworkParameters {
    RoadNetworkParameters() { checkParameterValidity(); }
    void checkParameterValidity() const;

    constexpr static const double eps1{
        0.1}; // param to reduce lateral distance of the projection domain border from the reference path
    constexpr static const double eps2{3.0};        // size of segments added to both ends of the reference path
    static const size_t stepsToResamplePolyline{2}; // steps for resampling polyline of the reference path
    static const int cornerCuttingRefinements{4};   // number of refinements of corner cutting
    constexpr static const double projectionDomainLimit{
        20.0}; // maximum absolute distance of the projection domain border from the reference path
    constexpr static const size_t numAdditionalSegmentsCCS{
        3};                                               // number of additional segments added at beginning to CCS
    size_t numIntersectionsPerDirectionLaneGeneration{1}; // number of intersections considered for lane generation
    size_t relevantTimeIntervalSize{
        100}; //**< relevant interval size around current time step to extract occupied lanelets in driving direction */
};
