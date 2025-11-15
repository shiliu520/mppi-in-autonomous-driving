#include <cassert>
#include <commonroad_cpp/predicates/predicate_parameter.h>
#include <tuple>

PredicateParam::PredicateParam(const std::string &name, const std::string &description, double max, double min,
                               const std::vector<std::string> &occurrences, const std::string &aProperty,
                               const std::string &usage, const std::string &type, const std::string &unit, double value)
    : name(name), description(description), max(max), min(min), occurrences(occurrences), property(aProperty),
      usage(usage), type(type), unit(unit), value(value) {}

void PredicateParam::updateValue(double val) { this->value = val; }

double PredicateParam::getValue() const { return value; }

void PredicateParam::checkParameterValidity() const {
    assert((name, value <= max));
    assert((name, value >= min));
}
std::tuple<std::string, std::string, double, double, std::vector<std::string>, std::string, std::string, std::string,
           std::string, double>
PredicateParam::asTuple() const {
    std::tuple<std::string, std::string, double, double, std::vector<std::string>, std::string, std::string,
               std::string, std::string, double>
        tmp;
    tmp = make_tuple(name, description, max, min, occurrences, property, usage, type, unit, value);
    return tmp;
}
