#include <string>
#include <stdio.h>
#include <future>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "Motrack.h"

namespace py = pybind11;

PYBIND11_MODULE(pymotrack, m)
{

    py::class_<motrack::Rect>(m, "Rect")
        .def(py::init<float, float, float, float>(),
            py::arg("x") = 0.0,
            py::arg("y") = 0.0,
            py::arg("width") = 0.0,
            py::arg("height") = 0.0)
		.def_property("x",        [](motrack::Rect& self){return self.x;}, [](motrack::Rect& self, float nv){self.x = nv;})
		.def_property("y",        [](motrack::Rect& self){return self.y;}, [](motrack::Rect& self, float nv){self.y = nv;})
        .def_property("width",    [](motrack::Rect& self){return self.width;}, [](motrack::Rect& self, float nv){self.width = nv;})
        .def_property("height",   [](motrack::Rect& self){return self.height;}, [](motrack::Rect& self, float nv){self.height = nv;})
		.def("__repr__", [](motrack::Rect& rect){
			auto str =std::string("\n") + "x:" + std::to_string(rect.x) + " "
					 + "y:" + std::to_string(rect.y) + " "
                     + "width:" + std::to_string(rect.width) + " "
                     + "height:" + std::to_string(rect.height);
			return str;
			}
		);

    py::class_<motrack::Object>(m, "Object")
        .def(py::init<float, unsigned int, const motrack::Rect&>(),
            py::arg("prob") = 0.0,
            py::arg("label") = 0,
            py::arg("rect") = motrack::Rect())
		.def_property("prob",        [](motrack::Object& self){return self.prob;}, [](motrack::Object& self, float nv){self.prob = nv;})
		.def_property("label",        [](motrack::Object& self){return self.label;}, [](motrack::Object& self, unsigned int nv){self.label = nv;})
        .def_property("rect",    [](motrack::Object& self){return self.rect;}, [](motrack::Object& self, motrack::Rect& nv){self.rect = nv;})
        .def("__repr__", [](motrack::Object& object){
            auto str =std::string("\n") + "prob:" + std::to_string(object.prob) + " "
					 + "label:" + std::to_string(object.label) + " "
                     + "x:" + std::to_string(object.rect.x) + " "
                     + "y:" + std::to_string(object.rect.y) + " "
                     + "width:" + std::to_string(object.rect.width) + " "
                     + "height:" + std::to_string(object.rect.height);
                     return str;
			}
		);

    py::class_<motrack::Track>(m, "Track")
        .def(py::init<bool, unsigned long long, unsigned long long, const motrack::Object&>(),
            py::arg("b_activated") = 0,
            py::arg("track_id") = 0,
            py::arg("frame_id") = 0,
            py::arg("object") = motrack::Object())
        .def_property("b_activated",        [](motrack::Track& self){return self.b_activated;}, [](motrack::Track& self, bool nv){self.b_activated = nv;})
        .def_property("track_id",        [](motrack::Track& self){return self.track_id;}, [](motrack::Track& self, unsigned long long nv){self.track_id = nv;})
        .def_property("frame_id",        [](motrack::Track& self){return self.frame_id;}, [](motrack::Track& self, unsigned long long nv){self.frame_id = nv;})
        .def_property("object",        [](motrack::Track& self){return self.object;}, [](motrack::Track& self, motrack::Object& nv){self.object = nv;})
        .def("__repr__", [](motrack::Track& track){
            auto str =std::string("\n") + "b_activated:" + std::to_string(track.b_activated) + " "
					 + "track_id:" + std::to_string(track.track_id) + " "
                     + "frame_id:" + std::to_string(track.frame_id) + " "
                     + "prob:" + std::to_string(track.object.prob) + " "
                     + "label:" + std::to_string(track.object.label) + " "
                     + "x:" + std::to_string(track.object.rect.x) + " "
                     + "y:" + std::to_string(track.object.rect.y) + " "
                     + "width:" + std::to_string(track.object.rect.width) + " "
                     + "height:" + std::to_string(track.object.rect.height);
                     return str;;
        });


    py::class_<motrack::TrackerConfig>(m, "TrackerConfig")
    .def(py::init<>())
    .def_readwrite("max_age", &motrack::TrackerConfig::max_age)
    .def_readwrite("track_thresh", &motrack::TrackerConfig::track_thresh)
    .def_readwrite("high_thresh", &motrack::TrackerConfig::high_thresh)
    .def_readwrite("match_thresh", &motrack::TrackerConfig::match_thresh)
    .def_readwrite("appearance_thresh", &motrack::TrackerConfig::appearance_thresh)
    .def_readwrite("lambda_weight", &motrack::TrackerConfig::lambda_weight)
    .def_readwrite("feature_dim", &motrack::TrackerConfig::feature_dim)
    .def_readwrite("feature_budget", &motrack::TrackerConfig::feature_budget);

    py::enum_<motrack::TrackerType>(m, "TrackerType")
    .value("ByteTrack", motrack::TrackerType::ByteTrack)
    .value("Sort", motrack::TrackerType::Sort)
    .value("OCSort", motrack::TrackerType::OCSort)
    .value("DeepSort", motrack::TrackerType::DeepSort)
    .value("JDE", motrack::TrackerType::JDE);

    py::class_<motrack::Tracker>(m, "Tracker")
    .def(py::init<motrack::TrackerType, const motrack::TrackerConfig&>(),
        py::arg("type"),
        py::arg("config") = motrack::TrackerConfig())
    .def("update", &motrack::Tracker::update,
        py::arg("objects"));
}