// SAPIEN 2 exposes setGlobalPose, but omits PhysX's kinematic motion target.
// Use the installed SAPIEN headers and its actor capsule; no object layout or
// hard-coded library symbols are assumed here.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sapien/sapien_actor_base.h>
#include <array>
#include <cmath>
#include <vector>
#include <extensions/PxJoint.h>
#include <extensions/PxConstraintExt.h>

namespace py = pybind11;

PYBIND11_MODULE(_rope_physx, module) {
  // Read only: no added sensors/joints, no solver or contact-filter changes.
  module.def("read_pair_constraint", [](py::capsule receiver, py::capsule other) {
    auto* a = static_cast<sapien::SActorBase*>(receiver.get_pointer());
    auto* b = static_cast<sapien::SActorBase*>(other.get_pointer());
    if (!a || !b || a == b) throw py::value_error("Two distinct actors are required");
    auto* pa = a->getPxActor();
    auto* pb = b->getPxActor();
    std::vector<physx::PxConstraint*> constraints(pa->getNbConstraints());
    const auto count = pa->getConstraints(constraints.data(), constraints.size());
    physx::PxConstraint* found = nullptr;
    bool first = false;
    for (unsigned i = 0; i < count; ++i) {
      physx::PxRigidActor *p0, *p1;
      constraints[i]->getActors(p0, p1);
      if ((p0 == pa && p1 == pb) || (p0 == pb && p1 == pa)) {
        if (found) throw py::value_error("More than one constraint connects this actor pair");
        found = constraints[i];
        first = p0 == pa;
      }
    }
    if (!found) throw py::value_error("No constraint connects this actor pair");
    physx::PxU32 owner;
    auto* reference = found->getExternalReference(owner);
    if (owner != physx::PxConstraintExtIDs::eJOINT)
      throw py::value_error("Constraint is not a PhysX joint");
    auto* joint = static_cast<physx::PxJoint*>(reference);
    physx::PxRigidActor *p0, *p1;
    found->getActors(p0, p1);
    const auto origin = (p1->getGlobalPose() * joint->getLocalPose(physx::PxJointActorIndex::eACTOR1)).p;
    physx::PxVec3 force, torque;
    found->getForce(force, torque);
    const float sign = first ? 1.f : -1.f;
    py::dict result;
    result["force"] = std::array<float, 3>{sign*force.x, sign*force.y, sign*force.z};
    result["torque"] = std::array<float, 3>{sign*torque.x, sign*torque.y, sign*torque.z};
    result["origin"] = std::array<float, 3>{origin.x, origin.y, origin.z};
    bool awake = false;
    for (auto* body : {pa, pb}) {
      if (auto* dynamic = body->is<physx::PxRigidDynamic>()) awake |= !dynamic->isSleeping();
      else if (auto* link = body->is<physx::PxArticulationLink>())
        awake |= !link->getArticulation().isSleeping();
    }
    result["awake"] = awake;
    return result;
  });
  // Isolated socket mounting reaction; null actor is the world, not USB.
  module.def("read_world_constraint", [](py::capsule receiver) {
    auto* a = static_cast<sapien::SActorBase*>(receiver.get_pointer());
    if (!a) throw py::value_error("Actor required");
    auto* pa = a->getPxActor();
    std::vector<physx::PxConstraint*> constraints(pa->getNbConstraints());
    auto count = pa->getConstraints(constraints.data(), constraints.size());
    physx::PxConstraint* found = nullptr;
    bool first = false;
    for (unsigned i=0; i<count; ++i) {
      physx::PxRigidActor *p0, *p1;
      constraints[i]->getActors(p0, p1);
      if ((p0 == pa && !p1) || (p1 == pa && !p0)) {
        if (found) throw py::value_error("Ambiguous world support");
        found = constraints[i]; first = p0 == pa;
      }
    }
    if (!found) throw py::value_error("Missing world support");
    physx::PxU32 owner;
    auto* ref = found->getExternalReference(owner);
    if (owner != physx::PxConstraintExtIDs::eJOINT) throw py::value_error("Not a joint");
    auto* joint = static_cast<physx::PxJoint*>(ref);
    auto local = joint->getLocalPose(first ? physx::PxJointActorIndex::eACTOR0 : physx::PxJointActorIndex::eACTOR1);
    auto origin = (pa->getGlobalPose()*local).p;
    physx::PxVec3 f, t; found->getForce(f, t);
    float sign = first ? 1.f : -1.f;
    py::dict result;
    result["force"] = std::array<float,3>{sign*f.x, sign*f.y, sign*f.z};
    result["torque"] = std::array<float,3>{sign*t.x, sign*t.y, sign*t.z};
    result["origin"] = std::array<float,3>{origin.x, origin.y, origin.z};
    auto* body = pa->is<physx::PxRigidDynamic>();
    result["awake"] = body && !body->isSleeping();
    return result;
  });
  module.def("disable_gravity", [](py::capsule capsule) {
    auto* actor = static_cast<sapien::SActorBase*>(capsule.get_pointer());
    auto* body = actor ? actor->getPxActor()->is<physx::PxRigidDynamic>() : nullptr;
    if (!body) throw py::value_error("Dynamic actor required");
    body->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, true);
  });
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
