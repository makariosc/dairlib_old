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
#include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
#include <drake/manipulation/kuka_iiwa/iiwa_status_sender.h>
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
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "systems/controllers/kuka_mbp_torque_controller.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "systems/vector_scope.h"

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
 using manipulation::kuka_iiwa::IiwaCommandReceiver;
 using manipulation::kuka_iiwa::IiwaStatusSender;
 using systems::StateInterpolatorWithDiscreteDerivative;
 using systems::ConstantVectorSource;

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

void SetTorqueControlledIiwaGains(Eigen::VectorXd* stiffness,
                                  Eigen::VectorXd* damping_ratio) {
  // All the gains are for directly generating torques. These gains are set
  // according to the values in the drake-iiwa-driver repository:
  // https://github.com/RobotLocomotion/drake-iiwa-driver/blob/master/kuka-driver/sunrise_1.11/DrakeFRITorqueDriver.java NOLINT

  // The spring stiffness in Nm/rad.
  stiffness->resize(7);
  *stiffness << 0, 0, 0, 0, 0, 0, 0;

  // A dimensionless damping ratio. See KukaTorqueController for details.
  damping_ratio->resize(stiffness->size());
  damping_ratio->setConstant(1.0);
}

int DoMain() {

  std::unique_ptr<multibody::MultibodyPlant<double>> owned_world_plant = std::make_unique<MultibodyPlant<double>>(0.002);
  std::unique_ptr<geometry::SceneGraph<double>> owned_scene_graph = std::make_unique<geometry::SceneGraph<double>>();

  multibody::MultibodyPlant<double>* world_plant = owned_world_plant.get();
  geometry::SceneGraph<double>* scene_graph = owned_scene_graph.get();
  world_plant->RegisterAsSourceForSceneGraph(scene_graph);

  std::unique_ptr<multibody::MultibodyPlant<double>> owned_controller_plant = std::make_unique<MultibodyPlant<double>>();

  // Adds a plant.
  const char* kModelPath =
  "drake/manipulation/models/iiwa_description/iiwa7/"
    "iiwa7_with_box_collision.sdf";
  const std::string kuka_urdf = FindResourceOrThrow(kModelPath);

  const auto X_WI = RigidTransform<double>::Identity();
  auto iiwa_instance = AddAndWeldModelFrom(kuka_urdf, "iiwa", world_plant->world_frame(), "iiwa_link_0", X_WI, world_plant);

  // Creates the plant to go in the simulation
  multibody::Parser parser(owned_controller_plant.get());
  const auto controller_iiwa_model = parser.AddModelFromFile(kuka_urdf, "iiwa");
  owned_controller_plant->WeldFrames(
   owned_controller_plant->world_frame(),
   owned_controller_plant->GetFrameByName("iiwa_link_0", controller_iiwa_model),
   X_WI);
  owned_controller_plant->Finalize();




  world_plant->Finalize();

  const int num_joints = world_plant->num_positions();

  const double kIiwaLcmStatusPeriod = 0.005;


  VectorX<double> q0_iiwa(num_joints);
  q0_iiwa << 0, 0, 0, 0, 0, 1.0, 0;

  // Set the iiwa default configuration.
  const auto iiwa_joint_indices =
      world_plant->GetJointIndices(iiwa_instance);
  int q0_index = 0;
  for (const auto joint_index : iiwa_joint_indices) {
    multibody::RevoluteJoint<double>* joint =
        dynamic_cast<multibody::RevoluteJoint<double>*>(
            &world_plant->get_mutable_joint(joint_index));
    // Note: iiwa_joint_indices includes the WeldJoint at the base.  Only set
    // the RevoluteJoints.
    if (joint) {
      joint->set_default_angle(q0_iiwa[q0_index++]);
    }
  }

  systems::DiagramBuilder<double> builder;
  builder.AddSystem(std::move(owned_world_plant));
  builder.AddSystem(std::move(owned_scene_graph));

  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();

  // Create the command subscriber and status publisher.
  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm));
  command_sub->set_name("command_subscriber");
  auto command_receiver =
      builder.AddSystem<IiwaCommandReceiver>(num_joints);
  command_receiver->set_name("command_receiver");
  auto desired_state_from_position = builder.AddSystem<
      StateInterpolatorWithDiscreteDerivative<double>>(
          num_joints, kIiwaLcmStatusPeriod);
  desired_state_from_position->set_name("desired_state_from_position");


  VectorX<double> stiffness, damping_ratio;
  SetTorqueControlledIiwaGains(&stiffness, &damping_ratio);
  damping_ratio = damping_ratio.replicate(1, 1);
  stiffness = stiffness.replicate(1, 1);

  auto iiwa_controller = builder.AddSystem<dairlib::systems::KukaTorqueController<double>>(std::move(owned_controller_plant), stiffness, damping_ratio);
  auto test = builder.AddSystem<dairlib::systems::VectorScope>(iiwa_controller->get_output_port_control().size());

  Eigen::VectorXd constTorqueValues(7);
  constTorqueValues << 0, 0, 0, 0, 0, 0, 0;

  auto constant_source = builder.AddSystem<ConstantVectorSource<double>>(constTorqueValues);

  // Creating status sender
  auto iiwa_status = builder.AddSystem<IiwaStatusSender>(num_joints);

  builder.Connect(command_sub->get_output_port(),
                        command_receiver->get_input_port());
  builder.Connect(command_receiver->get_commanded_position_output_port(),
                        desired_state_from_position->get_input_port());
  builder.Connect(desired_state_from_position->get_output_port(),
                        iiwa_controller->get_input_port_desired_state());

  builder.Connect(world_plant->get_state_output_port(),
                  iiwa_controller->get_input_port_estimated_state());
  builder.Connect(command_receiver->get_output_port(1),
                  iiwa_controller->get_input_port_commanded_torque());

  builder.Connect(iiwa_controller->get_output_port_control(),
                  test->get_input_port(0));

  builder.Connect(iiwa_controller->get_output_port_control(),
                  world_plant->get_actuation_input_port());

  // builder.Connect(constant_source->get_output_port(),
  //                 world_plant->get_actuation_input_port());

  builder.Connect(iiwa_controller->get_output_port_control(),
                  iiwa_status->get_position_commanded_input_port());

  // Demuxing system state for status publisher
  const int num_iiwa_positions = world_plant->num_positions();

  auto demux = builder.AddSystem<systems::Demultiplexer<double>>(
     2 * num_iiwa_positions, num_iiwa_positions);
  builder.Connect(
     world_plant->get_state_output_port(iiwa_instance), // Why is this so ridiculously large????
     demux->get_input_port(0));

  builder.Connect(demux->get_output_port(0),
                  iiwa_status->get_position_measured_input_port());
  builder.Connect(demux->get_output_port(1),
                  iiwa_status->get_velocity_estimated_input_port());

  builder.Connect(iiwa_controller->get_output_port_control(),
                  iiwa_status->get_torque_commanded_input_port());

  builder.Connect(iiwa_controller->get_output_port_control(),
                  iiwa_status->get_torque_measured_input_port());
  builder.Connect(world_plant->get_generalized_contact_forces_output_port(iiwa_instance),
                  iiwa_status->get_torque_external_input_port());

  auto iiwa_status_publisher = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm, 0.005 /* publish period */));
  builder.Connect(iiwa_status->get_output_port(),
                  iiwa_status_publisher->get_input_port());

  builder.Connect(
      world_plant->get_geometry_poses_output_port(),
      scene_graph->get_source_pose_port(world_plant->get_source_id().value()));
  builder.Connect(scene_graph->get_query_output_port(),
                  world_plant->get_geometry_query_input_port());

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
