import yaml


def generate_cpp_code(params):
    cpp_code = """#include <commonroad_cpp/predicates/predicate_parameter_collection.h>
#include <stdexcept>

void PredicateParameters::checkParameterValidity() {
    for (auto &[_, val] : parameterCollection) {
        val.checkParameterValidity();
    }
}

void PredicateParameters::updateParam(const std::string &name, double value) {
    if (parameterCollection.count(name) == 1) {
        parameterCollection.find(name)->second.updateValue(value);
        parameterCollection.find(name)->second.checkParameterValidity();
    } else
        throw std::runtime_error("No predicate " + name + " found for update");
}

double PredicateParameters::getParam(const std::string &name) {
    if (parameterCollection.count(name) == 1)
        return parameterCollection.find(name)->second.getValue();
    else if (constantMap.count(name) == 1)
        return constantMap.find(name)->second;
    else
        throw std::runtime_error("No predicate " + name + " found");
}

// when using a parameter in a new predicate or adding a new parameter, add it to the yaml file and generate the
// cpp via the Python script
std::map<std::string, PredicateParam> paramMap = {
"""
    for key, value in params.items():
        occurrences = ", ".join(['"' + str(i) + '"' for i in value["occurrences"]])
        cpp_code += (
            f'    {{"{key}", PredicateParam("{key}", "{value["description"]}", {value["max"]}, '
            f'{value["min"]}, {{{occurrences}}}, "{value["property"]}", "{value["strictness"]}", '
            f'"{value["type"]}", "{value["unit"]}", {value["value"]})}},\n'
        )
    cpp_code += "};"
    return cpp_code


with open("predicate_parameter.yaml", "r") as file:
    data = yaml.safe_load(file)

cpp_file_content = generate_cpp_code(data)

with open("predicate_parameter_collection.cpp", "w") as file:
    file.write(cpp_file_content)
