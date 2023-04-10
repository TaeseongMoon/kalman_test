#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "kalman_filter.h"

namespace py = pybind11;

PYBIND11_MODULE(kalman_filter, m) {
    py::class_<KalmanFilter>(m, "KalmanFilter")
        .def(py::init<int, int>())
        .def("initialize", &KalmanFilter::initialize)
        .def("predict", &KalmanFilter::predict)
        .def("update", &KalmanFilter::update)
        .def("get_state", &KalmanFilter::getState)
        .def("get_covariance", &KalmanFilter::getCovariance);
}