# script to generate yaml file from input file with lines of type
# {"aAbrupt", -2.0},        // acceleration difference which indicates abrupt braking [m/s^2]
# {"standstillError", 0.1}, // velocity deviation from zero which is still classified to be standstill [m/s^2]
import re
from pathlib import Path
from typing import Dict, List
import yaml


def search_cpp_files() -> Dict[str, List[str]]:
    # Dictionary to store results
    results = {}

    # Regular expression pattern to match 'parameters.getParam("name"]'
    pattern = re.compile(r'parameters\.getParam\("(.*?)"\)')

    # Iterate over all .cpp files in the current directory
    for filename in (Path(__file__).parent.parent / "src/commonroad_cpp/predicates").rglob("**/*.cpp"):
        with open(filename, "r") as file:
            # Read the file
            content = file.read()
            # Find all matches of the pattern
            matches = pattern.findall(content)
            # For each match, add the filename to the list of files for that match
            for match in matches:
                if match not in results:
                    results[match] = []
                if filename not in results[match]:
                    results[match].append(filename.stem[:-10])

    # Print the results
    return results


def match_property(prop: str) -> str:
    if "[m/s^3]" in prop:
        return "jerk"
    if "[m/s^2]" in prop:
        return "acceleration"
    if "[m/s]" in prop:
        return "velocity"
    if "[rad]" in prop:
        return "angle"
    if "[m]" in prop:
        return "distance"
    return "unknown"


def match_unit(input_string: str) -> str:
    # Find the start and end indices of the substring
    start_index = input_string.find("[") + 1
    end_index = input_string.find("]")
    # Extract and return the substring
    if start_index > 0 and end_index > start_index:
        return input_string[start_index:end_index]
    else:
        return "unknown"


def convert_to_yaml(input_file, output_file):
    yaml_dict = {}
    occurrences = search_cpp_files()
    with open(input_file, "r") as f:
        for line in f:
            # Remove whitespace and split on comma
            parts = line.strip().split(",")
            # Remove curly braces and quotation marks from key
            key = parts[0].replace("{", "").replace('"', "").strip()
            value_type = "float" if "." in parts[1][:-1] else "int"

            # Convert value to float
            try:
                value = float(parts[1][:-1]) if "." in parts[1][:-1] else int(parts[1][:-1])
            except ValueError:
                print(f"error {key}")
                value = 0.0
            # Extract description from comment
            description = parts[2].strip().lstrip("//").strip()
            # Add to dictionary with additional fields
            if occurrences.get(key):
                yaml_dict[key] = {
                    "value": value,
                    "description": description,
                    "type": value_type,
                    "min": -10.0 if key == "aAbrupt" else -50.0,
                    "max": 1.0 if key == "aAbrupt" else 3.0,
                    "strictness": "lower",
                    "occurrences": list(set(occurrences[key])),
                    "property": match_property(description),
                    "unit": match_unit(description),
                }

    with open(output_file, "w") as f:
        yaml.dump(yaml_dict, f, default_flow_style=False)


# Call the function with your input and output file paths
convert_to_yaml("input.txt", "predicate_parameter.yaml")
