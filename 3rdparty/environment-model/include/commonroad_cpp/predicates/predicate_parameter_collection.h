#pragma once

// Note: This include needs to come first in order to have M_PI defined
// See https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <commonroad_cpp/predicates/predicate_parameter.h>
#include <map>
#include <string>

extern std::map<std::string, PredicateParam> paramMap;

struct PredicateParameters {
    /**
     * Constructor of predicate parameters colllection.
     */
    PredicateParameters() { checkParameterValidity(); }

    /**
     * Checks validity of all parameters.
     */
    void checkParameterValidity();

    /**
     * Updates single predicate parameter
     *
     * @param name (string) name of parameter
     * @param value (double) value to be set
     */
    void updateParam(const std::string &name, double value);

    /**
     * Getter for predicate parameter
     *
     * @param name (string) name of parameter
     */
    double getParam(const std::string &name);

    /**
     * Getter for all predicate names.
     *
     * @return Vector of strings containing names.
     */
    [[nodiscard]] std::vector<std::string> getPredicateNames() const;

    /**
     * Getter for all constant names.
     *
     * @return Vector of strings containing names.
     */
    [[nodiscard]] std::vector<std::string> getConstantNames() const;

    /**
     * Returns all predicate parameters as map with parameter name as key and tuple as value.
     *
     * @return Map containing all predicates as tuple indexed by name.
     */
    [[nodiscard]] std::map<std::string, std::tuple<std::string, std::string, double, double, std::vector<std::string>,
                                                   std::string, std::string, std::string, std::string, double>>
    getParameterCollection() const;

    /**
     * Returns als updatable and constant parameters.
     *
     * @return Map containing all parameters.
     */
    std::map<std::string, double> getParameterCollectionComplete() const;

  private:
    std::map<std::string, double> constantMap{
        {"epsilon", 1e-6},         // small value close to zero for different purposes
        {"fovSpeedLimit", 50},     // field of view speed limit; will be replaced by compute with calc_v_max_fov() [m/s]
        {"brakingSpeedLimit", 50}, // braking speed limit; will be replaced by compute with calc_v_max_braking() [m/s]
        {"roadConditionSpeedLimit",
         50}, // road condition speed limit; will be replaced by compute with calc_v_max_road_condition() [m/s]
    };
    std::map<std::string, PredicateParam> parameterCollection = paramMap;
};
