#include "command_line_input.h"
#include "predicate_manager.h"
#include <iostream>

int main(int argc, char **argv) {

    // Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string filePath;
    if (const int error_code = InputUtils::readCommandLineValues(argc, argv, num_threads, filePath); error_code != 0)
        return error_code;
    try {
        PredicateManager eval{num_threads, filePath};
        eval.extractPredicateSatisfaction();
    } catch (...) {
        std::cout << "Undefined error env-model main function.";
    }
    return 0;
}
