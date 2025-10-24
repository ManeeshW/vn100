#include <pybind11/pybind11.h>
#include "vn100.hpp"

namespace py = pybind11;

PYBIND11_MODULE(vn100, m) {
    py::class_<vn100>(m, "vn100")
        .def(py::init<>())
        .def("load_config", &vn100::load_config, py::arg("config_path") = "")
        .def("open", py::overload_cast<const std::string&, int>(&vn100::open), py::arg("port"), py::arg("baud_rate"))
        .def("open", py::overload_cast<>(&vn100::open))
        .def("loop", &vn100::loop)
        .def("close", &vn100::close)
        .def("get_latest_data", [](vn100& self) -> py::tuple {
            vec3f ypr, ang_rate, accel;
            vec4f quat;
            bool success = self.get_latest_data(ypr, ang_rate, quat, accel);
            if (!success) {
                return py::make_tuple(false, py::none(), py::none(), py::none(), py::none());
            }
            auto to_tuple3 = [](const vec3f& v) {
                return py::make_tuple(v[0], v[1], v[2]);
            };
            auto to_tuple4 = [](const vec4f& v) {
                return py::make_tuple(v[0], v[1], v[2], v[3]);
            };
            return py::make_tuple(success, to_tuple3(ypr), to_tuple3(ang_rate), to_tuple4(quat), to_tuple3(accel));
        })
        .def_readwrite("port", &vn100::port)
        .def_readwrite("baud_rate", &vn100::baud_rate)
        .def_readwrite("frequency", &vn100::frequency)
        .def("enabled", &vn100::enabled);
}