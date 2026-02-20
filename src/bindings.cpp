#include "planner.hpp" 

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;



static InputShape parse_shape(const py::handle& obj) {
  // Expecting a dict: { "pose":[6], "scale":[3], "type": myompl.ShapeType.* }
  if (!py::isinstance<py::dict>(obj))
    throw std::runtime_error("shape must be a dict with keys: pose, scale, type");
  auto d = py::reinterpret_borrow<py::dict>(obj);

  auto pose_py  = d["pose"];
  auto scale_py = d["scale"];
  auto type_py  = d["type"];

  InputShape s;
  // pose
  {
    auto v = pose_py.cast<std::vector<double>>();
    if (v.size() != 6) throw std::runtime_error("shape.pose must have 6 elements");
    for (int i=0;i<6;++i) s.pose(i) = v[i];
  }
  // scale
  {
    auto v = scale_py.cast<std::vector<double>>();
    if (v.size() != 3) throw std::runtime_error("shape.scale must have 3 elements");
    for (int i=0;i<3;++i) s.scale(i) = v[i];
  }
  // type
  s.type = type_py.cast<InShapeType>();
  return s;
}

static std::vector<InputShape> parse_shape_list(const py::handle& obj) {
  std::vector<InputShape> out;
  if (obj.is_none()) return out;
  auto lst = py::reinterpret_borrow<py::sequence>(obj);
  out.reserve(py::len(lst));
  for (auto item : lst) out.push_back(parse_shape(item));
  return out;
}

// ------------ Aux dirs ------------
static std::array<Eigen::Vector3d,2> parse_aux(const py::handle& obj) {
  std::array<Eigen::Vector3d,2> a{};
  auto seq = py::reinterpret_borrow<py::sequence>(obj);
  if (py::len(seq) != 2) throw std::runtime_error("aux_dir must have 2 vec3s");
  for (int k=0;k<2;++k) {
    auto v = py::reinterpret_borrow<py::sequence>(seq[k]).cast<std::vector<double>>();
    if (v.size()!=3) throw std::runtime_error("each aux_dir entry must have 3 elements");
    for (int i=0;i<3;++i) a[k](i) = v[i];
  }
  return a;
}

// Convert std::vector<Eigen::VectorXd> -> numpy (N x D)
static py::array_t<double> to_numpy(const std::vector<Eigen::VectorXd>& wps) {
  if (wps.empty()) return py::array_t<double>(py::array::ShapeContainer{0, 0});
  const py::ssize_t N = static_cast<py::ssize_t>(wps.size());
  const py::ssize_t D = static_cast<py::ssize_t>(wps.front().size());
  py::array_t<double> arr({N, D});
  auto buf = arr.mutable_unchecked<2>();
  for (py::ssize_t i=0;i<N;++i) {
    if (static_cast<py::ssize_t>(wps[i].size()) != D)
      throw std::runtime_error("inconsistent waypoint dimension");
    for (py::ssize_t j=0;j<D;++j) buf(i,j) = wps[i](j);
  }
  return arr;
}



PYBIND11_MODULE(core, m) {
  m.doc() = "OMPL-based planner (pybind11) with simple dict-based scene input";

  const std::string self_file = py::str(m.attr("__file__"));
  g_pkg_dir = fs::path(self_file).parent_path().string();

  py::enum_<InShapeType>(m, "ShapeType")
    .value("Box",      InShapeType::Box)
    .value("Sphere",   InShapeType::Sphere)
    .value("Cylinder", InShapeType::Cylinder)
    .value("Mesh",     InShapeType::Mesh)
    .export_values();

  // Single function, kwargs-only style.
  m.def("plan",
    [](Eigen::VectorXd& start_joint,
       Eigen::VectorXd& goal_joint,
       Eigen::VectorXd& limit_n,
       Eigen::VectorXd& limit_p,
       py::object scene,
       py::object load,
       py::object gripper,
       const Eigen::Matrix<double,6,1>& tool,
       const Eigen::Matrix<double,6,1>& base_in_world,
       const Eigen::Matrix<double,6,1>& frame_in_world,
       py::object aux_dir,
       double time_limit_sec,
       int seed,
       bool has_camera,
       bool gravity,
       float gravity_thr)
    {
      if (start_joint.size() == 0) throw std::runtime_error("start_joint is empty");
      if (goal_joint.size()  != start_joint.size() || limit_n.size()  != start_joint.size() || limit_p.size()  != start_joint.size())
        throw std::runtime_error("goal_joint must match start_joint size");

      auto scene_v = parse_shape_list(scene);
      auto load_v  = parse_shape_list(load);
      auto gripper_v = parse_shape_list(gripper);
      auto aux     = parse_aux(aux_dir);

      auto wps = run_planner(start_joint, goal_joint, limit_n, limit_p, scene_v, load_v, gripper_v,
                             tool, base_in_world, frame_in_world, aux, time_limit_sec, g_pkg_dir, seed, has_camera, gravity, gravity_thr);
      return to_numpy(wps);
    },
    py::arg("start_joint"),
    py::arg("goal_joint"),
    py::arg("limit_n"),
    py::arg("limit_p"),
    py::arg("scene") = py::none(),
    py::arg("load")  = py::none(),
    py::arg("gripper") = py::none(),
    py::arg("tool"),
    py::arg("base_in_world"),
    py::arg("frame_in_world"),
    py::arg("aux_dir"),
    py::arg("time_limit_sec") = 1.0,
    py::arg("seed") = 1234,
    py::arg("has_camera") = false,
    py::arg("gravity") = false,
    py::arg("gravity_thr") = 1.0,
    R"doc(
plan(start_joint=..., goal_joint=..., scene=[{pose,scale,type},...], load=[...], gripper = [...]
     tool=[6], base_in_world=[6], frame_in_world=[6], aux_dir=[[3],[3]], time_limit_sec=1.0) -> np.ndarray (N, DOF)

- start_joint / goal_joint: 1D arrays; DOF inferred from length.
- scene / load: list of dicts with keys: pose(6), scale(3), type(path_planning.core.ShapeType.*).
- tool, base_in_world, frame_in_world: vec6 (x,y,z,rx,ry,rz).
- aux_dir: [vec3, vec3].
)doc");
}