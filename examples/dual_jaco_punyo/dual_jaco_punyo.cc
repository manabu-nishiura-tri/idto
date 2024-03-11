#include "examples/example_base.h"
#include "utils/find_resource.h"
#include <drake/geometry/proximity_properties.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");

namespace idto {
namespace examples {
namespace dual_jaco_punyo {

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

class DualJacoExample : public TrajOptExample {
 public:
  DualJacoExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(1.5, 0.0, 0.5);
    const Vector3d target_pose(0.0, 0.0, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // Add jaco arms
    std::string robot_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/j2s7s300_arm_without_nub_sphere_collision_v2.sdf");

    ModelInstanceIndex jaco_left = Parser(plant).AddModels(robot_file)[0];
    plant->RenameModelInstance(jaco_left, "jaco_left");
    RigidTransformd X_left(RollPitchYaw<double>(0, 0, M_PI_2),
                           Vector3d(0, 0.27, 0.114));
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base", jaco_left), X_left);
    plant->set_gravity_enabled(jaco_left, false);

    // Add left bubble paw
    std::string gripper_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/jaco_bubble_picoflexx_paw_hydro_sphere_collision.sdf");
    ModelInstanceIndex bubble_paw_left =
        Parser(plant).AddModels(gripper_file)[0];
    plant->RenameModelInstance(bubble_paw_left, "bubble_paw_left");
    RigidTransformd X_link7_lpaw(RollPitchYaw<double>(-1.57, 0., 2.09),
                                Vector3d(0.0, 0.0, 0.0));
    plant->WeldFrames(plant->GetFrameByName("j2s7s300_link_7", jaco_left),
                      plant->GetFrameByName("gripper", bubble_paw_left), X_link7_lpaw);
    // Disable gravity for bubble paw.
    plant->mutable_gravity_field().set_enabled(bubble_paw_left, false);

    ModelInstanceIndex jaco_right = Parser(plant).AddModels(robot_file)[0];
    plant->RenameModelInstance(jaco_right, "jaco_right");
    RigidTransformd X_right(RollPitchYaw<double>(0, 0, M_PI_2),
                            Vector3d(0, -0.27, 0.114));
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base", jaco_right), X_right);
    plant->set_gravity_enabled(jaco_right, false);

    // Add right bubble paw
    ModelInstanceIndex bubble_paw_right =
        Parser(plant).AddModels(gripper_file)[0];
    plant->RenameModelInstance(bubble_paw_right, "bubble_paw_right");
    RigidTransformd X_link7_rpaw(RollPitchYaw<double>(-1.57, 0., 2.09),
                                Vector3d(0.0, 0.0, 0.0));
    plant->WeldFrames(plant->GetFrameByName("j2s7s300_link_7", jaco_right),
                      plant->GetFrameByName("gripper", bubble_paw_right), X_link7_rpaw);
    // Disable gravity for bubble paw.
    plant->mutable_gravity_field().set_enabled(bubble_paw_right, false);

    // Add a manipuland
    std::string manipuland_file =
        idto::FindIdtoResourceOrThrow("idto/examples/models/rolled_yoga_mat_gaiam_sphere.sdf");
    Parser(plant).AddModels(manipuland_file);

    // Add the ground
    const drake::Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    RigidTransformd X_table(Vector3d(0.83, 0.0, -0.18));
    plant->RegisterVisualGeometry(plant->world_body(), X_table,
                                  Box(1.52, 1.52, 0.1), "table", tan);
    plant->RegisterCollisionGeometry(plant->world_body(), X_table,
                                     Box(1.52, 1.52, 0.1), "table",
                                     CoulombFriction<double>(0.5, 0.5));
  }

  void CreatePlantModelForSimulation(
      MultibodyPlant<double>* plant) const final {
    // Add jaco arms, including gravity
    std::string robot_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/j2s7s300_arm_without_nub_hydro_collision.sdf");

    ModelInstanceIndex jaco_left = Parser(plant).AddModels(robot_file)[0];
    plant->RenameModelInstance(jaco_left, "jaco_left");
    RigidTransformd X_left(RollPitchYaw<double>(0, 0, M_PI_2),
                           Vector3d(0, 0.27, 0.114));
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base", jaco_left), X_left);
    //plant->set_gravity_enabled(jaco_left, false);

    // Add left bubble paw
    std::string gripper_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/jaco_bubble_picoflexx_paw_hydro.sdf");
    ModelInstanceIndex bubble_paw_left =
        Parser(plant).AddModels(gripper_file)[0];
    plant->RenameModelInstance(bubble_paw_left, "bubble_paw_left");
    RigidTransformd X_link7_lpaw(RollPitchYaw<double>(-1.57, 0., 2.09),
                                Vector3d(0.0, 0.0, 0.0));
    plant->WeldFrames(plant->GetFrameByName("j2s7s300_link_7", jaco_left),
                      plant->GetFrameByName("gripper", bubble_paw_left), X_link7_lpaw);
    // Disable gravity for bubble paw.
    //plant->mutable_gravity_field().set_enabled(bubble_paw_left, false);

    ModelInstanceIndex jaco_right = Parser(plant).AddModels(robot_file)[0];
    plant->RenameModelInstance(jaco_right, "jaco_right");
    RigidTransformd X_right(RollPitchYaw<double>(0, 0, M_PI_2),
                            Vector3d(0, -0.27, 0.114));
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base", jaco_right), X_right);
    //plant->set_gravity_enabled(jaco_right, false);

    // Add right bubble paw
    ModelInstanceIndex bubble_paw_right =
        Parser(plant).AddModels(gripper_file)[0];
    plant->RenameModelInstance(bubble_paw_right, "bubble_paw_right");
    RigidTransformd X_link7_rpaw(RollPitchYaw<double>(-1.57, 0., 2.09),
                                Vector3d(0.0, 0.0, 0.0));
    plant->WeldFrames(plant->GetFrameByName("j2s7s300_link_7", jaco_right),
                      plant->GetFrameByName("gripper", bubble_paw_right), X_link7_rpaw);
    // Disable gravity for bubble paw.
    //plant->mutable_gravity_field().set_enabled(bubble_paw_right, false);
 
    // Add a manipuland with compliant hydroelastic contact
    std::string manipuland_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/rolled_yoga_mat_gaiam.sdf");
    Parser(plant).AddModels(manipuland_file);

    // Add the ground with compliant hydroelastic contact
    const drake::Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);

    std::string table_file = idto::FindIdtoResourceOrThrow(
        "idto/examples/models/symphony_table.sdf");
    RigidTransformd X_table(Vector3d(0.87, 0.0, -0.1));
    ModelInstanceIndex table = Parser(plant).AddModels(table_file)[0];
    plant->RenameModelInstance(table, "table");
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("table_top_center"), X_table);
    /*
    plant->RegisterVisualGeometry(plant->world_body(), X_table, Box(1.52, 1.52, 0.1),
                                  "table", tan);

    ProximityProperties ground_proximity;
    AddContactMaterial({}, {}, CoulombFriction<double>(0.5, 0.5),
                       &ground_proximity);
    //AddCompliantHydroelasticProperties(0.1, 5e7, &ground_proximity);
    plant->RegisterCollisionGeometry(plant->world_body(), X_table,
                                     Box(1.52, 1.52, 0.1), "table",
                                     ground_proximity);
    */
  }
};

}  // namespace dual_jaco_punyo
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::dual_jaco_punyo::DualJacoExample example;
  example.RunExample("idto/examples/dual_jaco_punyo/dual_jaco_punyo.yaml", FLAGS_test);

  return 0;
}
