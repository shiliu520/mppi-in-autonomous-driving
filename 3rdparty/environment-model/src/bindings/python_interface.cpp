#include <memory>

#include "python_interface_core.h"
// #include "python_interface_predicates.h"

namespace nb = nanobind;

NB_MODULE(_crcpp, m) {
    m.doc() = "CommonRoad Python/C++ Interface";

    // nb::module_ m_predicates = m.def_submodule("predicates");
    // init_python_interface_predicates(m_predicates);

    init_python_interface_core(m);
}
