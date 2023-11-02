#include "Iisy11r1300Module.h"

#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace
{

// This is set by CMake, see CMakeLists.txt
static const std::string MC_IISY11_R1300_DESCRIPTION_PATH = "@MC_IISY11_R1300_DESCRIPTION_PATH@";

} // namespace

namespace mc_robots
{

Iisy11r1300Module::Iisy11r1300Module() : mc_rbdyn::RobotModule(MC_IISY11_R1300_DESCRIPTION_PATH, "iisy11_r1300")
{
  // True if the robot has a fixed base, false otherwise
  bool fixed = true;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));

  // Automatically load the convex hulls associated to each body
  std::string convexPath = path + "/convex/" + name + "/";
  bfs::path p(convexPath);
  if(bfs::exists(p) && bfs::is_directory(p))
  {
    std::vector<bfs::path> files;
    std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
    for(const bfs::path & file : files)
    {
      size_t off = file.filename().string().rfind("-ch.txt");
      if(off != std::string::npos)
      {
        std::string name = file.filename().string();
        name.replace(off, 7, "");
        _convexHull[name] = std::pair<std::string, std::string>(name, file.string());
      }
    }
  }

  // Define some force sensors
  _forceSensors.push_back(mc_rbdyn::ForceSensor("EndEffectorForceSensor", "joint_a6", sva::PTransformd::Identity()));


  // Define a minimal set of self-collisions
  _minimalSelfCollisions = {
      {"base_link_inertia", "link_2", 0.02, 0.001, 0.}, 
      {"base_link_inertia", "link_4", 0.02, 0.001, 0.}, 
      {"base_link_inertia", "link_5", 0.02, 0.001, 0.}, 
      {"base_link_inertia", "link_6", 0.02, 0.001, 0.}, 
      {"link_1", "link_4", 0.02, 0.001, 0.},
      {"link_1", "link_5", 0.02, 0.001, 0.},
      {"link_1", "link_6", 0.02, 0.001, 0.},
      {"link_2", "link_4", 0.02, 0.001, 0.},
      {"link_2", "link_5", 0.02, 0.001, 0.},
      {"link_2", "link_6", 0.02, 0.001, 0.},
      {"link_4", "link_6", 0.02, 0.001, 0.}};
  _commonSelfCollisions = _minimalSelfCollisions;

  // Define simple grippers
  // _grippers = {{"l_gripper", {"L_UTHUMB"}, true}, {"r_gripper", {"R_UTHUMB"}, false}};


  // Default configuration of the floating base
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.0}};

  // Default joint configuration, if a joint is omitted the configuration is 0 or the middle point of the limit range if
  // 0 is not a valid configuration
  _stance["joint_a1"] = {0.01};
  _stance["joint_a2"] = {0.01};
  _stance["joint_a3"] = {0.01};
  _stance["joint_a4"] = {0.01};
  _stance["joint_a5"] = {0.01};
  _stance["joint_a6"] = {0.01};
}

} // namespace mc_robots

#include <mc_rbdyn/RobotModuleMacros.h>

ROBOT_MODULE_DEFAULT_CONSTRUCTOR("iisy11_r1300", mc_robots::Iisy11r1300Module)
