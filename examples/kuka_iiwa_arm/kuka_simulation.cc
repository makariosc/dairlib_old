/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual robot hardware.

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/dev/render/render_engine_vtk.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/sensors/dev/rgbd_camera.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

 using Eigen::Vector3d;
 using Eigen::VectorXd;
 using geometry::SceneGraph;
 using math::RigidTransform;
 using math::RollPitchYaw;
 using math::RotationMatrix;
 using multibody::Joint;
 using multibody::MultibodyPlant;
 using multibody::PrismaticJoint;
 using multibody::RevoluteJoint;
 using multibody::SpatialInertia;

// Load a SDF model and weld it to the MultibodyPlant.
// @param model_path Full path to the sdf model file. i.e. with
// FindResourceOrThrow
// @param model_name Name of the added model instance.
// @param parent Frame P from the MultibodyPlant to which the new model is
// welded to.
// @param child_frame_name Defines frame C (the child frame), assumed to be
// present in the model being added.
// @param X_PC Transformation of frame C relative to frame P.
template <typename T>
multibody::ModelInstanceIndex AddAndWeldModelFrom(
    const std::string& model_path, const std::string& model_name,
    const multibody::Frame<T>& parent, const std::string& child_frame_name,
    const RigidTransform<double>& X_PC, MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(!plant->HasModelInstanceNamed(model_name));

  multibody::Parser parser(plant);
  const multibody::ModelInstanceIndex new_model =
      parser.AddModelFromFile(model_path, model_name);
  const auto& child_frame = plant->GetFrameByName(child_frame_name, new_model);
  plant->WeldFrames(parent, child_frame, X_PC);
  return new_model;
}

int DoMain() {

  std::unique_ptr<multibody::MultibodyPlant<double>> owned_world_plant = std::make_unique<MultibodyPlant<double>>(0.002);
  std::unique_ptr<geometry::SceneGraph<double>> owned_scene_graph = std::make_unique<geometry::SceneGraph<double>>();
  std::unique_ptr<multibody::MultibodyPlant<double>> owned_controller_plant = std::make_unique<MultibodyPlant<double>>();

  multibody::MultibodyPlant<double>* world_plant = owned_world_plant.get();
  geometry::SceneGraph<double>* scene_graph = owned_scene_graph.get();
  world_plant->RegisterAsSourceForSceneGraph(scene_graph);

  systems::DiagramBuilder<double> builder;

  // Create the command subscriber and status publisher.
  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm));
  command_sub->set_name("command_subscriber");
  auto command_receiver =
      builder.AddSystem<IiwaCommandReceiver>(num_joints);
  command_receiver->set_name("command_receiver");



  // Adds a plant.
  const char* kModelPath =
  "drake/manipulation/models/iiwa_description/iiwa7/"
      "iiwa7_no_collision.sdf";
  const std::string kuka_urdf = FindResourceOrThrow(kModelPath);

  const auto X_WI = RigidTransform<double>::Identity();
  AddAndWeldModelFrom(kuka_urdf, "iiwa", world_plant->world_frame(), "iiwa_link_0", X_WI, world_plant);

  world_plant->Finalize();

  builder.AddSystem(std::move(owned_world_plant));
  builder.AddSystem(std::move(owned_scene_graph));

  auto iiwa_

  builder.Connect(
      world_plant->get_geometry_poses_output_port(),
      scene_graph->get_source_pose_port(world_plant->get_source_id().value()));
  builder.Connect(scene_graph->get_query_output_port(),
                  plant->get_geometry_query_input_port());

  geometry::ConnectDrakeVisualizer(&builder, *scene_graph, scene_graph->get_pose_bundle_output_port());

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());
  return 0;

}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::DoMain();
}
