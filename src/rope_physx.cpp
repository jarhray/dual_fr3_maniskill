// SAPIEN 2 exposes setGlobalPose, but omits PhysX's kinematic motion target.
// Use the installed SAPIEN headers and its actor capsule; no object layout or
// hard-coded library symbols are assumed here.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sapien/sapien_actor_base.h>
#include <array>
#include <cmath>

namespace py = pybind11;

PYBIND11_MODULE(_rope_physx, module) {
  module.def("set_max_depenetration_velocity", [](py::capsule capsule, float speed) {
    auto* actor = static_cast<sapien::SActorBase*>(capsule.get_pointer());
    auto* body = actor ? actor->getPxActor()->is<physx::PxRigidBody>() : nullptr;
    if (!body) throw py::value_error("A SAPIEN rigid body is required");
    if (!std::isfinite(speed) || speed <= 0)
      throw py::value_error("Depenetration speed must be positive and finite");
    body->setMaxDepenetrationVelocity(speed);
  });
  module.def("get_max_depenetration_velocity", [](py::capsule capsule) {
    auto* actor = static_cast<sapien::SActorBase*>(capsule.get_pointer());
    auto* body = actor ? actor->getPxActor()->is<physx::PxRigidBody>() : nullptr;
    if (!body) throw py::value_error("A SAPIEN rigid body is required");
    return body->getMaxDepenetrationVelocity();
  });
  module.def("set_kinematic_target", [](py::capsule capsule,
      const std::array<float, 3>& p, const std::array<float, 4>& q) {
    auto* actor = static_cast<sapien::SActorBase*>(capsule.get_pointer());
    if (!actor || (actor->getType() != sapien::EActorType::KINEMATIC &&
                   actor->getType() != sapien::EActorType::KINEMATIC_ARTICULATION_LINK))
      throw py::value_error("A SAPIEN kinematic actor is required");
    const physx::PxTransform target(physx::PxVec3(p[0], p[1], p[2]),
                                  physx::PxQuat(q[1], q[2], q[3], q[0]));
    if (!target.isValid()) throw py::value_error("Invalid kinematic target");
    static_cast<physx::PxRigidDynamic*>(actor->getPxActor())->setKinematicTarget(target);
  });
}
