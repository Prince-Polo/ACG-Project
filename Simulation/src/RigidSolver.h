#pragma once
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include "BaseContainer.h"

// RigidSolver 类用于处理刚体物理求解，类似于 PyBullet 的刚体求解
class RigidSolver {
public:
    // 构造函数，传入容器、时间步长和重力加速度
    RigidSolver(BaseContainer& container, double dt, Eigen::Vector3d gravity);

    // 析构函数
    ~RigidSolver();

    // 插入一个刚体物体
    void insertRigidObject();

    // 每个时间步进行求解
    void step();

    // 应用力到粒子
    void applyForceToParticle(int particleId, const Eigen::Vector3d& force);

    // 应用力矩到粒子
    void applyTorqueToParticle(int particleId, const Eigen::Vector3d& torque);

private:
    // 创建边界，厚度默认为 0.01
    void createBoundary(double thickness = 0.01);

    // 初始化刚体
    void initializeRigidBody(const RigidBody& rigidBody);

    // 初始化刚体块
    void initializeRigidBlock(const RigidBlock& rigidBlock);

    // 计算极分解
    Eigen::Matrix3d computePolarDecomposition(const Eigen::Matrix3d& matrix);

    // 对象容器
    BaseContainer& container;

    // 时间步长
    double dt;

    // 总时间
    double totalTime;

    // 重力加速度
    Eigen::Vector3d gravity;

    // 存储当前存在的刚体物体
    std::vector<int> presentRigidObjects;

    // 存储刚体的旋转矩阵
    std::unordered_map<int, Eigen::Matrix3d> rotationMatrices;

    // 存储刚体的质心
    std::unordered_map<int, Eigen::Vector3d> centersOfMass;

    // 存储刚体的速度
    std::unordered_map<int, Eigen::Vector3d> velocities;

    // 存储刚体的角速度
    std::unordered_map<int, Eigen::Vector3d> angularVelocities;
};
