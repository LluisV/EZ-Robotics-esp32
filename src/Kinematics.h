#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <ArduinoEigen.h>
#include "DH.h"

/**
 * @brief Computes the forward kinematics of the robot, obtaining the full transformation matrix.
 *
 * @param robot The DH object representing the robot.
 * @return Eigen::Matrix4f The 4x4 transformation matrix representing the end-effector position and orientation.
 */
Eigen::Matrix4f forward_kinematics(const DH& robot);


/**
 * @brief Calculates the robot's inverse kinematics using an iterative method based on the pseudo-inverse of the Jacobian.
 *
 * @param robot DH object representing the robot. The joint positions will be updated internally.
 * @param desiredTransform 4x4 matrix representing the desired position and orientation of the end effector.
 * @param solution Vector where the found solution (joint positions) will be stored.
 * @param tolerance Convergence tolerance for the error (default is 1e-3).
 * @param maxIterations Maximum number of iterations (default is 100).
 * @return true if convergence is reached; false if the maximum number of iterations is exceeded.
 */
bool inverse_kinematics(DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    Eigen::VectorXf &solution,
    float tolerance = 1e-3,
    int maxIterations = 1000);

/**
 * @brief Calculates all possible IK solutions for robots with analytical solutions.
 *
 * @param robot DH object representing the robot.
 * @param desiredTransform Desired end-effector transformation.
 * @param solutions Vector of all possible joint configurations.
 * @return int Number of valid solutions found.
 */
int inverse_kinematics_analytical(
    const DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    std::vector<Eigen::VectorXf> &solutions);

/**
 * @brief Analytical IK for SCARA robots.
 *
 * @param robot DH object representing the SCARA robot.
 * @param desiredTransform Desired end-effector transformation.
 * @param solutions Vector to store the solutions (up to 2).
 * @return int Number of valid solutions found.
 */
int inverse_kinematics_scara(
    const DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    std::vector<Eigen::VectorXf> &solutions);

/**
 * @brief Analytical IK for Cartesian robots.
 *
 * @param robot DH object representing the Cartesian robot.
 * @param desiredTransform Desired end-effector transformation.
 * @param solution Solution vector.
 * @return bool True if solution found.
 */
bool inverse_kinematics_cartesian(
    const DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    Eigen::VectorXf &solution);

/**
 * @brief Analytical IK for 2R planar robots.
 *
 * @param robot DH object representing the 2R robot.
 * @param desiredTransform Desired end-effector transformation.
 * @param solutions Vector to store the solutions (up to 2).
 * @return int Number of valid solutions found.
 */
int inverse_kinematics_2r(
    const DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    std::vector<Eigen::VectorXf> &solutions);

/**
 * @brief Select the best IK solution based on current joint configuration.
 *
 * @param currentJoints Current joint configuration.
 * @param solutions Vector of possible solutions.
 * @return int Index of the best solution, or -1 if no valid solution.
 */
int selectBestIKSolution(
    const Eigen::VectorXf &currentJoints,
    const std::vector<Eigen::VectorXf> &solutions);

/**
 * @brief Compute joint distance metric for solution selection.
 *
 * @param joints1 First joint configuration.
 * @param joints2 Second joint configuration.
 * @return float Distance metric.
 */
float computeJointDistance(
    const Eigen::VectorXf &joints1,
    const Eigen::VectorXf &joints2);

#endif // KINEMATICS_H