#pragma once

namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::BasicVector;

class VectorScope : public drake::systems::LeafSystem<double> {
 public:
  VectorScope(int size);

 private:
  // const std::string msg_;

  void PrintOutput(const Context<double>& context, BasicVector<double>* notreallyoutput) const;

};

} // namespace systems
} // namespace dairlib
