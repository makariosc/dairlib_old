#define WORLDFRAME_ID 0

#include "systems/controllers/mbp_endeffector_position_controller.h"

namespace dairlib{
namespace systems{

EndEffectorPositionController::EndEffectorPositionController(
    const MultibodyPlant<double>& plant, int ee_frame_id,
    Eigen::Vector3d ee_contact_frame, int num_joints, double k_p, double k_omega)
    : plant_local(plant){

  // Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
  joint_position_measured_port = this->DeclareVectorInputPort(
      "joint_position_measured", BasicVector<double>(num_joints)).get_index();
  endpoint_position_commanded_port = this->DeclareVectorInputPort(
      "endpoint_position_commanded", BasicVector<double>(3)).get_index();
  endpoint_orientation_commanded_port = this->DeclareVectorInputPort(
      "endpoint_orientation_commanded", BasicVector<double>(4)).get_index();
  endpoint_position_cmd_output_port = this->DeclareVectorOutputPort(
      BasicVector<double>(6), &EndEffectorPositionController::CalcOutputTwist).get_index();

  // The coordinates for the end effector with respect to the last joint.
  // Eventually passed into transformPointsJacobian()
  this->ee_contact_frame = ee_contact_frame;
  // frame ID of the joint connected to the end effector
  this->ee_frame_id = ee_frame_id;
  this->k_p = k_p;
  this->k_omega = k_omega;
}

void EndEffectorPositionController::CalcOutputTwist(
    const Context<double> &context, BasicVector<double>* output) const {

  VectorX<double> q_actual = this->EvalVectorInput(context,
      joint_position_measured_port)->CopyToVector();

 // std::cout << "q_actual_position" << std::endl;
 // std::cout << q_actual << std::endl;

  VectorX<double> x_desired = this->EvalVectorInput(context,
      endpoint_position_commanded_port)->CopyToVector();

  VectorX<double> orientation_desired = this->EvalVectorInput(context,
      endpoint_orientation_commanded_port)->CopyToVector();


  Eigen::Vector3d x_actual;
  const std::unique_ptr<Context<double>> plant_context = plant_local.CreateDefaultContext();
  plant_local.SetPositions(plant_context.get(), q_actual);
  // std::cout << "postiions: " << std::endl;
  // std::cout << plant_local.GetPositions(*plant_context) << std::endl;
  // std::cout << "q_actual" << std::endl;
  // std::cout << q_actual << std::endl;
  plant_local.CalcPointsPositions(*plant_context, plant_local.GetFrameByName("iiwa_link_7"), ee_contact_frame, plant_local.world_frame(), &x_actual);

  MatrixXd diff = k_p * (x_desired - x_actual);

  // Quaternion for rotation from base to end effector
  Eigen::Quaternion<double> quat_n_a = plant_local.CalcRelativeTransform(*plant_context, plant_local.world_frame(), plant_local.GetFrameByName("iiwa_link_7")).rotation().ToQuaternion();

  std::cout << "quat_n_a" << std::endl;
  std::cout << quat_n_a.w();
  std::cout << ", ";
  std::cout << quat_n_a.x();
  std::cout << ", ";
  std::cout << quat_n_a.y();
  std::cout << ", ";
  std::cout << quat_n_a.z() << std::endl;

  // Quaternion for rotation from world frame to desired end effector attitude.
  Eigen::Quaternion<double> quat_n_a_des = Eigen::Quaternion<double>(
      orientation_desired(0), orientation_desired(1), orientation_desired(2),
      orientation_desired(3));

      std::cout << "quat_n_a_des" << std::endl;
      std::cout << quat_n_a_des.w();
      std::cout << ", ";
      std::cout << quat_n_a_des.x();
      std::cout << ", ";
      std::cout << quat_n_a_des.y();
      std::cout << ", ";
      std::cout << quat_n_a_des.z() << std::endl;

  // Quaternion for rotation from end effector attitude to desired end effector attitude.
  Eigen::Quaternion<double> quat_a_a_des = quat_n_a.conjugate().operator*(quat_n_a_des);

  // Angle Axis Representation for the given quaternion
  Eigen::AngleAxis<double> angleaxis_a_a_des = Eigen::AngleAxis<double>(quat_a_a_des);
  MatrixXd axis = angleaxis_a_a_des.axis();
  MatrixXd angularVelocity = k_omega * axis * angleaxis_a_a_des.angle();

  // Transforming angular velocity from joint frame to world frame
  MatrixXd angularVelocityWF = plant_local.CalcRelativeTransform(*plant_context, plant_local.GetFrameByName("iiwa_link_7"), plant_local.world_frame()).rotation() * angularVelocity;

  std::cout << x_desired << std::endl;
  std::cout << "@@@@@@@@@@@@@@@@@@" << std::endl;
  std::cout << x_actual << std::endl;

  MatrixXd twist(6, 1);
  twist << angularVelocityWF, diff;
  // std::cout << "twist:" << std::endl;
  // std::cout << twist << std::endl;
  output->set_value(twist);
}

} // namespace systems
} // namespace dairlib
