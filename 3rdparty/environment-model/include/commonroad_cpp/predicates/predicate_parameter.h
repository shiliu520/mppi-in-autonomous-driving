#pragma once

#include <string>
#include <vector>

/**
 * Class representing a predicate parameter.
 */
class PredicateParam {
  public:
    /**
     * PredicateParam constructor
     * @param name Name of parameter.
     * @param description Textual description of parameter.
     * @param max Maximum possible value of parameter.
     * @param min Minimum possible value of parameter.
     * @param occurrences Predicates where the parameter is used.
     * @param property State element for which parameter is used, e.g., acceleration, velocity, etc.
     * @param usage Type of usage, greater means " < parameter", less means " > parameter", none means that the
     * parameter is not directly compared or used in both directions.
     * @param type Type of parameter, e.g., float or int
     * @param unit SI-Unit of parameter.
     * @param value Default value of parameter.
     */
    PredicateParam(const std::string &name, const std::string &description, double max, double min,
                   const std::vector<std::string> &occurrences, const std::string &property, const std::string &usage,
                   const std::string &type, const std::string &unit, double value);

    /**
     * Updates value.
     * @param val New value.
     */
    void updateValue(double val);

    /**
     * Getter of value.
     * @return Value as double.
     */
    [[nodiscard]] double getValue() const;

    /**
     * Checks validity of parameter, i.e., that value is within min/max.
     */
    void checkParameterValidity() const;

    /**
     * Creates a tuple containing the class variables.
     *
     * @return Tuple containing the class variables.
     */
    [[nodiscard]] std::tuple<std::string, std::string, double, double, std::vector<std::string>, std::string,
                             std::string, std::string, std::string, double>
    asTuple() const;

  private:
    std::string name;                     //**<Name of parameter.*/
    std::string description;              //**<Textual description of parameter.*/
    double max;                           //**<Maximum possible value of parameter.*/
    double min;                           //**<Minimum possible value of parameter.*/
    std::vector<std::string> occurrences; //**<Predicates where the parameter is used.*/
    std::string property; //**<State element for which parameter is used, e.g., acceleration, velocity, etc.*/
    std::string usage; //**<Type of usage, greater means " < parameter", less means " > parameter", none means that the
                       // parameter is not directly compared or used in both directions.*/
    std::string type;  //**<Type of parameter, e.g., float or int*/
    std::string unit;  //**<SI-Unit of parameter.*/
    double value;      //**<Default value of parameter.*/
};
