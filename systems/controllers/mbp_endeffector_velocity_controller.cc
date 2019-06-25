#include "systems/controllers/mbp_endeffector_velocity_controller.h"

namespace dairlib{
namespace systems{

EndEffectorVelocityController::EndEffectorVelocityController(
    const MultibodyPlant<double>& plant, Eigen::Vector3d ee_contact_frame,
    int num_joints, double k_d, double k_r) : plant(plant){

  // Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
  joint_position_measured_port = this->DeclareVectorInputPort(
      "joint_position_measured", BasicVector<double>(num_joints)).get_index();
  joint_velocity_measured_port = this->DeclareVectorInputPort(
      "joint_velocity_measured", BasicVector<double>(num_joints)).get_index();
  endpoint_twist_commanded_port = this->DeclareVectorInputPort(
      "endpoint_twist_commanded", BasicVector<double>(6)).get_index();

  // Note that this function contains a pointer to the callback function below.
  endpoint_torque_output_port = this->DeclareVectorOutputPort(
      BasicVector<double>(num_joints),
      &EndEffectorVelocityController::CalcOutputTorques).get_index();

  this->eeContactFrame = ee_contact_frame;
  this->num_joints = num_joints;
  this->k_d = k_d;
  this->k_r = k_r;
}

// Callback for DeclareVectorInputPort. No return value. The parameter 'output' is the output.
// This function is called many times a second, if I understand correctly.
void EndEffectorVelocityController::CalcOutputTorques(
  const Context<double>& context, BasicVector<double>* output) const {
  // We read the above input ports with EvalVectorInput
  // The purpose of CopyToVector().head(NUM_JOINTS) is to remove the timestamp from the vector input ports
  VectorX<double> q = this->EvalVectorInput(context,
      joint_position_measured_port)->CopyToVector().head(num_joints);

  VectorX<double> q_dot = this->EvalVectorInput(context,
      joint_velocity_measured_port)->CopyToVector().head(num_joints);

  VectorX<double> twist_desired = this->EvalVectorInput(context,
      endpoint_twist_commanded_port)->CopyToVector();

  const std::unique_ptr<Context<double>> plant_context = plant.CreateDefaultContext();
  plant.SetPositions(plant_context.get(), q);
  plant.SetVelocities(plant_context.get(), q_dot);

  // Calculating the jacobian of the kuka arm
  Eigen::Matrix<double, 6, 7> frameSpatialVelocityJacobian;
  plant.CalcFrameGeometricJacobianExpressedInWorld(*plant_context, plant.GetFrameByName("iiwa_link_7"), eeContactFrame, &frameSpatialVelocityJacobian);

  // Using the jacobian, calculating the actual current velocities of the arm
  MatrixXd twist_actual = frameSpatialVelocityJacobian * q_dot;

  // Gains are placed in a diagonal matrix
  Eigen::DiagonalMatrix<double, 6> gains(6);
  gains.diagonal() << k_r, k_r, k_r, k_d, k_d, k_d;

  // Calculating the error
  MatrixXd generalizedForces = gains * (twist_desired - twist_actual);

  std::cout << "frameSpatialVelocityJacobian" << std::endl;
  std::cout << frameSpatialVelocityJacobian << std::endl;
  // std::cout << "q_dot" << std::endl;
  // std::cout << q_dot << std::endl;
  // std::cout << "generalizedForces:" << std::endl;
  // std::cout << generalizedForces << std::endl;

  VectorXd manualout(7);
  manualout << 0, 0, 0, 0, 0, 0, 0;

  VectorXd outTorques(7);
  outTorques = frameSpatialVelocityJacobian.transpose() * generalizedForces;

  double joint_torque_limit = 0.5;
  // Limit maximum commanded velocities
  for(int i = 0; i < 7; i++) {
      double currSpeed = outTorques(i, 0);
      if (outTorques(i, 0) > joint_torque_limit) {
          outTorques(i, 0) = joint_torque_limit;
          std::cout << "Warning: velocity of component " << i <<  " limited from " << currSpeed << " to " << joint_torque_limit << std::endl;
      }
  }

  // Multiplying J^t x force to get torque outputs, then storing them in the output vector
  output->set_value(outTorques); // (7 x 6) * (6 x 1) = 7 x 1
  // output->set_value(manualout);
  // std::cout << "torques" << std::endl;
  // std::cout << frameSpatialVelocityJacobian.transpose() * generalizedForces << std::endl;
  // Getting last element for timestep since timestep is stored in last element
  //output->set_timestamp(this->EvalVectorInput(context, joint_position_measured_port)->GetAtIndex(NUM_JOINTS));
}

} // namespace systems
} // namespace dairlib
