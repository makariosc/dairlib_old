#include "systems/controllers/iiwa_velocity_controller.h"

namespace dairlib{
namespace systems{

KukaIiwaVelocityController::KukaIiwaVelocityController(const std::string urdf,
                                                       Eigen::Isometry3d eeCFIsometry,
                                                       int num_joints, int k_d,
                                                       int k_r) {
  // Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
  joint_position_measured_port = this->DeclareVectorInputPort("joint_position_measured", BasicVector<double>(num_joints)).get_index();
  joint_velocity_measured_port = this->DeclareVectorInputPort("joint_velocity_measured", BasicVector<double>(num_joints)).get_index();
  endpoint_twist_commanded_port = this->DeclareVectorInputPort("endpoint_twist_commanded", BasicVector<double>(6)).get_index();

  // Note that this function contains a pointer to the callback function below.
  this->DeclareVectorOutputPort(BasicVector<double>(num_joints), &KukaIiwaVelocityController::CalcOutputTorques);

  // initialize a rigidbodytree, and initialize the urdf specified in the parameters for it
  tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(urdf, drake::multibody::joints::kFixed, tree.get());

  this->eeCFIsometry = eeCFIsometry;
  this->num_joints = num_joints;
  this->k_d = k_d;
  this->k_r = k_r;
}

// Callback for DeclareVectorInputPort. No return value. The parameter 'output' is the output.
// This function is called many times a second, if I understand correctly.
void KukaIiwaVelocityController::CalcOutputTorques(const Context<double>& context, BasicVector<double>* output) const
{
  // We read the above input ports with EvalVectorInput
  // The purpose of CopyToVector().head(NUM_JOINTS) is to remove the timestamp from the vector input ports
  VectorX<double> q = this->EvalVectorInput(context, joint_position_measured_port)->CopyToVector().head(num_joints);
  VectorX<double> q_dot = this->EvalVectorInput(context, joint_velocity_measured_port)->CopyToVector().head(num_joints);
  VectorX<double> twist_desired = this->EvalVectorInput(context, endpoint_twist_commanded_port)->CopyToVector();

  // Calculating the jacobian of the kuka arm
  KinematicsCache<double> cache = tree->doKinematics(q, q_dot);
  MatrixXd frameSpatialVelocityJacobian = tree->CalcFrameSpatialVelocityJacobianInWorldFrame(cache, tree->get_body(10), eeCFIsometry);

  // Using the jacobian, calculating the actual current velocities of the arm
  MatrixXd generalizedVelocity_actual = frameSpatialVelocityJacobian * q_dot;

  // Gains are placed in a diagonal matrix
  Eigen::DiagonalMatrix<double, 6> gains(6);
  gains.diagonal() << k_r, k_r, k_r, k_d, k_d, k_d;

  // Calculating the error
  MatrixXd generalizedForces = gains * (twist_desired - generalizedVelocity_actual);

  // Multiplying J^t x force to get torque outputs, then storing them in the output vector
  output->set_value(frameSpatialVelocityJacobian.transpose() * generalizedForces); // (7 x 6) * (6 x 1) = 7 x 1

  // Getting last element for timestep since timestep is stored in last element
  //output->set_timestamp(this->EvalVectorInput(context, joint_position_measured_port)->GetAtIndex(NUM_JOINTS));
}

} // namespace systems
} // namespace dairlib
