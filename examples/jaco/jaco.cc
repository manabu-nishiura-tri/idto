#include "examples/example_base.h"
#include "utils/find_resource.h"
#include <drake/geometry/proximity_properties.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");
DEFINE_string(trajectory, "/home/manabunishiura/planning_through_contact/jaco_push_refined_trj.yaml",
              "GQDP trajectory file used to feed initial guess of IDTO");
DEFINE_double(nominal_update_dt, 0.4,
              "Interval for updating the nominal_state.");
DEFINE_string(yaml_file, "idto/examples/jaco/jaco.yaml",
              "Configuration file containing the definition of the trajectory "
              "optimization problem");
DEFINE_bool(time_varying_cost, false,
            "When set to true, the trajectory optimization process takes into "
            "account costs that vary over time, allowing for a more dynamic "
            "and responsive optimization. If set to false, the optimization "
            "assumes constant costs throughout the trajectory, leading to a "
            "usual optimization approach.");


struct SavedTrajectory {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(x_trj));
    a->Visit(DRAKE_NVP(u_trj));
    a->Visit(DRAKE_NVP(x_trj_intersegment_refined));
    a->Visit(DRAKE_NVP(playback_trajectory));
  }

  double dt{0};

  std::vector<Eigen::VectorXd> x_trj;
  std::vector<Eigen::VectorXd> u_trj;
  std::vector<Eigen::VectorXd> x_trj_intersegment_refined;
  std::vector<Eigen::VectorXd> playback_trajectory;
};

std::vector<Eigen::VectorXd> ExtractTrajectory(
    const std::vector<Eigen::VectorXd>& trajectory, int num_joints) {
  std::vector<Eigen::VectorXd> desired_trajectory;
  for (const Eigen::VectorXd& step : trajectory) {
    desired_trajectory.push_back(step.head(num_joints));
  }
  return desired_trajectory;
}

std::vector<Eigen::VectorXd> ExtractInitialTrajectory(
    const std::vector<Eigen::VectorXd>& trajectory, int num_joints,
    int num_steps) {
  std::vector<Eigen::VectorXd> desired_trajectory;
  int traj_length = 0;
  for (const Eigen::VectorXd& step : trajectory) {
    if (traj_length < num_steps) {
        desired_trajectory.push_back(step.head(num_joints));
        traj_length += 1;
    }
    else {break;}
  }
  return desired_trajectory;
}


using idto::examples::TrajOptExampleParams;
using drake::yaml::LoadYamlFile;
using drake::yaml::LoadYamlOptions;

namespace idto {
namespace examples {
namespace jaco {

using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::math::RollPitchYaw;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

class JacoExample : public TrajOptExample {
 public:
  JacoExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(1.5, 0.0, 0.5);
    const Vector3d target_pose(0.0, 0.0, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // Add a jaco arm without gravity
    std::string robot_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/j2s7s300_arm_sphere_collision_v2.sdf");
    ModelInstanceIndex jaco = Parser(plant).AddModels(robot_file)[0];
    RigidTransformd X_jaco(RollPitchYaw<double>(0, 0, M_PI_2),
                           Vector3d(0, 0, 0.11));
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"),
                      X_jaco);
    plant->set_gravity_enabled(jaco, false);

    // Add a manipuland with sphere contact
    std::string manipuland_file =
        idto::FindIdtoResourceOrThrow("idto/examples/models/box_intel_nuc.sdf");
    Parser(plant).AddModels(manipuland_file);

    // Add the ground
    const drake::Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -0.5));
    RigidTransformd X_table(Vector3d(0.6, 0.0, -0.499));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground, Box(25, 25, 1),
                                  "ground", green);
    plant->RegisterVisualGeometry(plant->world_body(), X_table,
                                  Box(1.5, 1.5, 1), "table", tan);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 1), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }

  void CreatePlantModelForSimulation(
      MultibodyPlant<double>* plant) const final {
    // Use hydroelastic contact, and throw instead of point contact fallback
    plant->set_contact_model(drake::multibody::ContactModel::kHydroelastic);

    // Add a jaco arm, including gravity, with rigid hydroelastic contact
    std::string robot_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/j2s7s300_arm_hydro_collision.sdf");
    ModelInstanceIndex jaco = Parser(plant).AddModels(robot_file)[0];
    RigidTransformd X_jaco(RollPitchYaw<double>(0, 0, M_PI_2),
                           Vector3d(0, 0., 0.11));
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"),
                      X_jaco);
    plant->set_gravity_enabled(jaco, false);

    // Add a manipuland with compliant hydroelastic contact
    std::string manipuland_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/box_intel_nuc_hydro.sdf");
    Parser(plant).AddModels(manipuland_file);

    // Add the ground with compliant hydroelastic contact
    const drake::Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -0.5));
    RigidTransformd X_table(Vector3d(0.6, 0.0, -0.499));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground, Box(25, 25, 1),
                                  "ground", green);
    plant->RegisterVisualGeometry(plant->world_body(), X_table,
                                  Box(1.5, 1.5, 1), "table", tan);

    ProximityProperties ground_proximity;
    AddContactMaterial(3.0, {}, CoulombFriction<double>(0.5, 0.5),
                       &ground_proximity);
    AddCompliantHydroelasticProperties(0.1, 5e6, &ground_proximity);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 1), "ground",
                                     ground_proximity);
  }
};

}  // namespace jaco
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Load the problem definition from YAML
  TrajOptExampleParams default_options;
  TrajOptExampleParams options =
      LoadYamlFile<TrajOptExampleParams>(FLAGS_yaml_file, {}, default_options);

  LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;
  yaml_options.allow_cpp_with_no_yaml = true;
  SavedTrajectory gqdp_trajectory_;
  gqdp_trajectory_ =
    LoadYamlFile<SavedTrajectory>(FLAGS_trajectory, {}, {}, yaml_options);
  std::vector<Eigen::VectorXd> initial_trajectory;
  std::vector<Eigen::VectorXd> whole_trajectory;
  const int num_joints = gqdp_trajectory_.u_trj.front().size();
  whole_trajectory = ExtractTrajectory(gqdp_trajectory_.playback_trajectory,
                                        num_joints + 7);
  initial_trajectory = ExtractInitialTrajectory(gqdp_trajectory_.playback_trajectory,
                                        num_joints + 7,
                                        options.num_steps + 1);

  idto::examples::jaco::JacoExample example;
  example.RunExample("idto/examples/jaco/jaco.yaml", initial_trajectory,
            whole_trajectory, FLAGS_nominal_update_dt, FLAGS_test,
            FLAGS_time_varying_cost);

  return 0;
}
