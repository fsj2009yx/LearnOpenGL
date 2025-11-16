
#ifndef PHYSICS_H
#define PHYSICS_H

#include <iostream>
#include <future>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
#include "body.h"


//牛顿引力常数等常用参数
inline float dt;
inline constexpr double GRAV_CONST = 6.67430e-11;
inline const glm::vec3 GRAV_FORCE = glm::vec3(0.0f, 0.0f, 0.0f);
inline constexpr double EPSILON = 1e-3;

class Physics {
public:
    /**
     * @brief 物理引擎默认构造函数
     *
     * 初始化物理引擎，使用以下默认参数：
     * - Speed: 3.0 (simulation speed multiplier)
     * - Timestep: 1/60 seconds (60 FPS fixed timestep)
     * - Simulation state: active (endSim = false)
     */
    Physics();

    /**
     * @brief Construct physics engine with custom simulation speed.
     *
     * @param speed Simulation speed multiplier (1.0 = normal, >1.0 = faster, <1.0 = slower)
     */
    Physics(float speed);

    /**
     * @brief Construct physics engine with custom timestep and speed.
     *
     * Allows full control over both temporal accuracy and simulation speed.
     * Smaller timesteps improve accuracy but increase computational cost.
     *
     * @param timeStep Physics update interval in seconds (e.g., 1/120 = 120 FPS physics)
     * @param speed Simulation speed multiplier applied to velocity calculations
     */
    Physics(float timeStep, float speed);

    // 对物体施加一个瞬时冲量（改变速度）
    static void push(Body &sphere, glm::vec3 force);

    void wait(float sec);

    /**
     * @brief Execute one physics timestep for all bodies in the simulation.
     *
     * This is the main physics loop that performs:
     * 1. Velocity integration: v += a * dt
     * 2. Position integration: p += v * dt * speed
     * 3. Exponential velocity damping: v *= e^(-λ*dt) (simulates drag/friction)
     * 4. Exponential acceleration damping: a *= e^(-λ*dt) (force decay)
     * 5. Boundary checking: terminate simulation if body crosses threshold
     *
     * Uses Euler integration for simplicity. Future versions may implement
     * RK4 or Verlet integration for improved numerical stability.
     *
     * @param bodies Reference to vector of all Body objects in the simulation
     */
    void processFrame(std::vector<Body *> bodies);

    /**
     * @brief Check if the simulation should terminate.
     *
     * Returns true when any body crosses the defined boundary threshold
     * or when another termination condition is met.
     *
     * @return true if simulation should stop, false otherwise
     */
    bool shouldClose();

    /**
     * @brief Clean up physics engine resources.
     *
     * Currently a placeholder for future cleanup operations
     * (e.g., memory deallocation, file writing, logging).
     */
    void cleanup();

private:
    // Simulation parameters
    float Speed; ///< Global speed multiplier for all motion
    bool endSim; ///< Flag to terminate simulation when boundary reached

    // 判断向量是否接近零向量
    bool isZero(glm::vec3 &vector);

    // 根据力和加速度更新物体的速度和位置（Euler积分）
    void updateState(Body &body);

    float calculateDistanceSquare(Body &sphereOne, Body &sphereTwo);

    // 计算两物体之间的万有引力并累加到力向量
    void calculateGravForce(Body &sphereOne, Body &sphereTwo);

    //计算物体受到的总力
    void calculateForce(Body &body);

    // 判断物体是否接触地面/表面
    bool onSurface(Body &body);

    // 处理物体表面碰撞
    void processSurfaceCollision(Body &body);


    // 判断两物体是否碰撞
    bool areColliding(Body &sphereOne, Body &sphereTwo);

    // 处理两物体弹性碰撞
    void processCollision(Body &sphereOne, Body &sphereTwo);

    /**
     * @brief Calculate Euclidean distance between centers of two bodies.
     *
     * Uses squared distance internally to avoid expensive sqrt operation
     * until necessary. Distance is calculated as: d = √((p₁-p₂)⋅(p₁-p₂))
     *
     * @param sphereOne First body
     * @param sphereTwo Second body
     * @return Distance between body centers in world units
     */
    double getDistance(Body &sphereOne, Body &sphereTwo);
};

#endif
