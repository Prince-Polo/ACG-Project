#include "RigidSolver.h"
#include <iostream>

RigidSolver::RigidSolver(BaseContainer& container, double dt, Eigen::Vector3d gravity)
    : container(container), dt(dt), gravity(gravity), totalTime(0.0) {}

void RigidSolver::step() {
    for (int particleId = 0; particleId < container.getParticleCount(); ++particleId) {
        if (container.isDynamicRigidBody(particleId)) {
            Eigen::Vector3d force = container.getRigidBodyForce(particleId);
            Eigen::Vector3d torque = container.getRigidBodyTorque(particleId);

            applyForceToParticle(particleId, force);
            applyTorqueToParticle(particleId, torque);

            // Reset forces and torques
            container.setRigidBodyForce(particleId, Eigen::Vector3d::Zero());
            container.setRigidBodyTorque(particleId, Eigen::Vector3d::Zero());

            // Update rigid body states
            Eigen::Vector3d velocity = velocities[particleId] + dt * force / container.getRigidBodyMass(particleId);
            Eigen::Vector3d angularVelocity = angularVelocities[particleId] + dt * torque; // Assuming unit inertia

            centersOfMass[particleId] += dt * velocity;
            Eigen::Matrix3d rotation = Eigen::AngleAxisd(angularVelocity.norm() * dt, angularVelocity.normalized()).toRotationMatrix();
            rotationMatrices[particleId] = rotation * rotationMatrices[particleId];

            velocities[particleId] = velocity;
            angularVelocities[particleId] = angularVelocity;

            container.setRigidBodyCenterOfMass(particleId, centersOfMass[particleId]);
            container.setRigidBodyRotationMatrix(particleId, rotationMatrices[particleId]);
        }
    }

    totalTime += dt;
}

void RigidSolver::insertRigidObject() {
    for (const auto& rigidBody : container.getRigidBodies()) {
        initializeRigidBody(rigidBody);
    }
    for (const auto& rigidBlock : container.getRigidBlocks()) {
        initializeRigidBlock(rigidBlock);
    }
}

void RigidSolver::applyForceToParticle(int particleId, const Eigen::Vector3d& force) {
    // Apply force to rigid body
    if (container.isDynamicRigidBody(particleId)) {
        velocities[particleId] += dt * force / container.getRigidBodyMass(particleId);
    }
}

void RigidSolver::applyTorqueToParticle(int particleId, const Eigen::Vector3d& torque) {
    // Apply torque to rigid body
    if (container.isDynamicRigidBody(particleId)) {
        angularVelocities[particleId] += dt * torque; // Assuming unit inertia
    }
}

void RigidSolver::createBoundary(double thickness) {
    // Boundary logic here (e.g., create virtual walls for simulation space)
    // You can use container domain information to setup boundary constraints.
    std::cout << "Boundary created with thickness: " << thickness << std::endl;
}

void RigidSolver::initializeRigidBody(const RigidBody& rigidBody) {
    if (std::find(presentRigidObjects.begin(), presentRigidObjects.end(), rigidBody.objectId) != presentRigidObjects.end()) {
        return;
    }

    presentRigidObjects.push_back(rigidBody.objectId);
    centersOfMass[rigidBody.objectId] = rigidBody.translation;
    rotationMatrices[rigidBody.objectId] = Eigen::Matrix3d::Identity();
    velocities[rigidBody.objectId] = rigidBody.isDynamic ? rigidBody.velocity : Eigen::Vector3d::Zero();
    angularVelocities[rigidBody.objectId] = Eigen::Vector3d::Zero();
}

void RigidSolver::initializeRigidBlock(const RigidBlock& rigidBlock) {
    // TODO: Add logic for initializing rigid blocks
    std::cerr << "Rigid block initialization not implemented!" << std::endl;
}

Eigen::Matrix3d RigidSolver::computePolarDecomposition(const Eigen::Matrix3d& matrix) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}
