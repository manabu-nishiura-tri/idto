#include "traj_opt/examples/example_base.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "common/find_resource.h"

namespace idto {
namespace traj_opt {
namespace examples {
namespace kuka {

using drake::geometry::Box;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;

class KukaExample : public TrajOptExample {
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    const drake::Vector4<double> blue(0.1, 0.3, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    const drake::Vector4<double> black(0.0, 0.0, 0.0, 1.0);

    // Add a kuka arm
    std::string robot_file = drake::FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_spheres_collision.urdf");
    ModelInstanceIndex kuka = Parser(plant).AddModelFromFile(robot_file);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"));
    // plant->disable_gravity(kuka);

    // Add a manipuland
    std::string manipuland_file = idto::FindIDTOResourceOrThrow(
        "traj_opt/examples/models/box_intel_nuc.sdf");
    Parser(plant).AddAllModelsFromFile(manipuland_file);

    // Add the ground
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -5.0));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground,
                                  Box(25, 25, 10), "ground", green);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 10), "ground",
                                     CoulombFriction<double>(0.5, 0.5));
  }
};

}  // namespace kuka
}  // namespace examples
}  // namespace traj_opt
}  // namespace idto

int main() {
  idto::traj_opt::examples::kuka::KukaExample example;
  example.RunExample("traj_opt/examples/kuka.yaml");
  return 0;
}