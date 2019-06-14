#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/basic_vector.h"

#include "systems/vector_scope.h"

namespace dairlib {
namespace systems {

using drake::systems::BasicVector;
using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::kVectorValued;

VectorScope::VectorScope(int size) {

    this->DeclareVectorInputPort("debug: ", BasicVector<double>(size));
    this->DeclareVectorOutputPort(BasicVector<double>(1),
                                  &VectorScope::PrintOutput);
}

void VectorScope::PrintOutput(const Context<double>& context, BasicVector<double>* notreallyoutput) const {
    auto val = this->EvalVectorInput(context, 0)->get_value();
    std::cout << val << std::endl;
    std::cout << "hello hello" << std::endl;

    Eigen::VectorXd notOut(1);
    notOut << 42;
    notreallyoutput->set_value(notOut);
}

} // namespace systems
} // namespace dairlib
