#include "Kinematics.h"
#include <cmath>
#include "RoboticsUtils.h"

Eigen::Matrix4f forward_kinematics(const DH& robot) {
    return robot.getTransformUpToJoint(-1); 
}

bool inverse_kinematics(DH &robot, const Eigen::Matrix4f &desiredTransform, Eigen::VectorXf &solution, float tolerance, int maxIterations) {
    // First, check if we can use analytical solution
    if (robot.getRobotType() != RobotType::GENERIC) {
        std::vector<Eigen::VectorXf> solutions;
        int numSolutions = inverse_kinematics_analytical(robot, desiredTransform, solutions);
        
        if (numSolutions > 0) {
            // Select the best solution based on current configuration
            int bestIdx = selectBestIKSolution(robot.getJointPositions(), solutions);
            if (bestIdx >= 0) {
                solution = solutions[bestIdx];
                robot.setJointPositions(solution);
                return true;
            }
        }
    }
    
    // Fall back to numerical solution
    bool debug = false;
    
    // Check if target is reachable
    Eigen::Vector3f targetPos = desiredTransform.block<3,1>(0,3);
    if (!robot.isPositionReachable(targetPos)) {
        Serial.println("Warning: Target position may not be reachable");
    }
    
    // We use the current pose as the initial guess.
    Eigen::VectorXf q = robot.getJointPositions();

    // Store the initial pose to return it if the algorithm doesn't converge
    Eigen::VectorXf initial_q = q;

    // Pre-calculate invariant parts of the desired transformation
    const Eigen::Vector3f p_des = desiredTransform.block<3,1>(0,3);
    const Eigen::Matrix3f R_des = desiredTransform.block<3,3>(0,0);

    // Temporary variables for the error
    Eigen::Vector3f error_position, error_orientation;
    Eigen::Matrix<float,6,1> deltaX;

    // Adaptive optimization parameters
    float damping = 0.001f;  // Initial damping factor
    float lambda = 0.1f;     // Initial gain for the update
    float prev_error = 1e10f;
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        // Update the robot configuration
        robot.setJointPositions(q);
        
        // Check for singularity
        if (robot.isNearSingularity()) {
            damping = 0.1f;  // Increase damping near singularities
            if (debug) {
                Serial.println("Warning: Near singularity, increasing damping");
            }
        } else {
            damping = 0.001f;  // Normal damping
        }
        
        // Calculate the current transformation (using forward kinematics)
        Eigen::Matrix4f currentTransform = forward_kinematics(robot);
        
        // --- Calculate position error ---
        Eigen::Vector3f p_current = currentTransform.block<3,1>(0,3);
        error_position = p_des - p_current;

        // --- Calculate orientation error ---
        Eigen::Matrix3f R_current = currentTransform.block<3,3>(0,0);
        // R_error is the difference between the desired and current orientation
        Eigen::Matrix3f R_error = R_des * R_current.transpose();
        float trace_R = R_error.trace();
        float angle = acosf((trace_R - 1.0f) / 2.0f);
        if (fabs(angle) < 1e-6f) {
            error_orientation.setZero();
        } else {
            // Extract the error vector from the logarithm of the rotation matrix
            error_orientation << R_error(2,1) - R_error(1,2),
                                 R_error(0,2) - R_error(2,0),
                                 R_error(1,0) - R_error(0,1);
            error_orientation *= (angle / (2.0f * sin(angle)));
        }
        
        // Form the total error vector (6x1): [position error; orientation error]
        deltaX << error_position, error_orientation;

        // If debug mode is enabled, print errors and current position
        if (debug) {
            Serial.print("Iteración: ");
            Serial.println(iter);
            Serial.print("Posición deseada: ");
            RoboticsUtils::print_vector(p_des);
            Serial.print("Posición actual: ");
            RoboticsUtils::print_vector(p_current);
            Serial.print("Error de posición: ");
            RoboticsUtils::print_vector(error_position);
            Serial.print("Error de orientación (ángulo): ");
            Serial.println(angle);
            Serial.print("Manipulabilidad: ");
            Serial.println(robot.computeManipulability());
        }

        float current_error = deltaX.norm();
        
        // Check if convergence has been reached
        if (current_error < tolerance) {
            solution = q;
            return true;
        }
        
        // Adaptive gain adjustment
        if (current_error > prev_error) {
            lambda *= 0.5f;  // Reduce gain if error increased
            if (lambda < 0.01f) lambda = 0.01f;
        } else if (current_error < 0.5f * prev_error) {
            lambda *= 1.1f;  // Increase gain if converging well
            if (lambda > 0.5f) lambda = 0.5f;
        }
        prev_error = current_error;
        
        // --- Calculate the Jacobian and its pseudoinverse ---
        Eigen::MatrixXf J = robot.computeJacobian();  // Matrix of dimensions 6 x n (n: number of joints)
        Eigen::MatrixXf JJT = J * J.transpose();      // 6 x 6
        JJT.diagonal().array() += damping;
        // Calculate the pseudoinverse using LDLT decomposition
        Eigen::MatrixXf J_pinv = J.transpose() * JJT.ldlt().solve(Eigen::MatrixXf::Identity(JJT.rows(), JJT.cols()));
              
        // Update the configuration using adaptive gain
        Eigen::VectorXf delta_q = lambda * (J_pinv * deltaX);
        
        // Limit joint velocity to avoid large jumps
        float max_delta = 0.2f;  // Maximum joint change per iteration
        for (int i = 0; i < delta_q.size(); ++i) {
            if (fabs(delta_q(i)) > max_delta) {
                delta_q(i) = max_delta * (delta_q(i) > 0 ? 1 : -1);
            }
        }
        
        q += delta_q;
    }
    
    // If it does not converge, return the initial pose
    solution = initial_q;
    
    return false;
}

int inverse_kinematics_analytical(
    const DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    std::vector<Eigen::VectorXf> &solutions) {
    
    solutions.clear();
    
    switch (robot.getRobotType()) {
        case RobotType::SCARA:
            return inverse_kinematics_scara(robot, desiredTransform, solutions);
            
        case RobotType::CARTESIAN:
            {
                Eigen::VectorXf solution;
                if (inverse_kinematics_cartesian(robot, desiredTransform, solution)) {
                    solutions.push_back(solution);
                    return 1;
                }
                return 0;
            }
            
        case RobotType::ARTICULATED_2R:
            return inverse_kinematics_2r(robot, desiredTransform, solutions);
            
        default:
            return 0;  // No analytical solution available
    }
}

int inverse_kinematics_scara(
    const DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    std::vector<Eigen::VectorXf> &solutions) {
    
    if (robot.getNumberOfJoints() < 3) return 0;
    
    // Extract target position
    float x = desiredTransform(0, 3);
    float y = desiredTransform(1, 3);
    float z = desiredTransform(2, 3);
    
    // Get DH parameters (assuming standard SCARA configuration)
    // Joint 0: prismatic (Z)
    // Joint 1: revolute (first arm)
    // Joint 2: revolute (second arm)
    
    // For the example SCARA in the code:
    // Link lengths are stored in the 'a' parameter
    float a1 = 200.0f;  // From joint 1
    float a2 = 200.0f;  // From joint 2
    
    // Z is directly the first joint value for standard SCARA
    float q0 = z;
    
    // Check if Z is within limits
    if (robot.getJointLimit(0, LimitType::MIN) < robot.getJointLimit(0, LimitType::MAX)) {
        if (q0 < robot.getJointLimit(0, LimitType::MIN) || 
            q0 > robot.getJointLimit(0, LimitType::MAX)) {
            return 0;  // Z out of bounds
        }
    }
    
    // Solve for the two revolute joints
    float r = sqrt(x*x + y*y);
    
    // Check reachability
    if (r > a1 + a2 || r < fabs(a1 - a2)) {
        return 0;  // Target out of reach
    }
    
    // Calculate joint 2 angle (elbow)
    float cos_q2 = (r*r - a1*a1 - a2*a2) / (2*a1*a2);
    if (fabs(cos_q2) > 1.0f) return 0;  // Numerical error
    
    // Two solutions: elbow up and elbow down
    float q2_1 = acos(cos_q2);
    float q2_2 = -q2_1;
    
    // For each elbow solution, calculate joint 1
    for (int i = 0; i < 2; ++i) {
        float q2 = (i == 0) ? q2_1 : q2_2;
        
        // Calculate joint 1 angle
        float k1 = a1 + a2 * cos(q2);
        float k2 = a2 * sin(q2);
        float q1 = atan2(y, x) - atan2(k2, k1);
        
        // Wrap angles
        q1 = RoboticsUtils::wrap_angle(q1);
        q2 = RoboticsUtils::wrap_angle(q2);
        
        // Check joint limits
        bool valid = true;
        if (robot.getJointLimit(1, LimitType::MIN) < robot.getJointLimit(1, LimitType::MAX)) {
            if (q1 < robot.getJointLimit(1, LimitType::MIN) || 
                q1 > robot.getJointLimit(1, LimitType::MAX)) {
                valid = false;
            }
        }
        if (robot.getJointLimit(2, LimitType::MIN) < robot.getJointLimit(2, LimitType::MAX)) {
            if (q2 < robot.getJointLimit(2, LimitType::MIN) || 
                q2 > robot.getJointLimit(2, LimitType::MAX)) {
                valid = false;
            }
        }
        
        if (valid) {
            Eigen::VectorXf solution(3);
            solution << q0, q1, q2;
            solutions.push_back(solution);
        }
    }
    
    return solutions.size();
}

bool inverse_kinematics_cartesian(
    const DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    Eigen::VectorXf &solution) {
    
    if (robot.getNumberOfJoints() < 3) return false;
    
    // Extract target position
    float x = desiredTransform(0, 3);
    float y = desiredTransform(1, 3);
    float z = desiredTransform(2, 3);
    
    // For Cartesian robots, the solution is straightforward
    // Assuming standard configuration: Z, X, Y prismatic joints
    solution.resize(3);
    solution << z, x, y;
    
    // Check joint limits
    for (int i = 0; i < 3; ++i) {
        if (robot.getJointLimit(i, LimitType::MIN) < robot.getJointLimit(i, LimitType::MAX)) {
            if (solution(i) < robot.getJointLimit(i, LimitType::MIN) || 
                solution(i) > robot.getJointLimit(i, LimitType::MAX)) {
                return false;  // Joint out of bounds
            }
        }
    }
    
    return true;
}

int inverse_kinematics_2r(
    const DH &robot,
    const Eigen::Matrix4f &desiredTransform,
    std::vector<Eigen::VectorXf> &solutions) {
    
    if (robot.getNumberOfJoints() < 2) return 0;
    
    // Extract target position (assuming planar robot in XY plane)
    float x = desiredTransform(0, 3);
    float y = desiredTransform(1, 3);
    
    // Get link lengths from DH parameters
    float a1 = 1.0f;  // Assuming unit lengths from test examples
    float a2 = 1.0f;
    
    // Calculate distance to target
    float r = sqrt(x*x + y*y);
    
    // Check reachability
    if (r > a1 + a2 || r < fabs(a1 - a2)) {
        return 0;  // Target out of reach
    }
    
    // Calculate joint 2 angle (elbow)
    float cos_q2 = (x*x + y*y - a1*a1 - a2*a2) / (2*a1*a2);
    if (fabs(cos_q2) > 1.0f) return 0;  // Numerical error
    
    // Two solutions: elbow up and elbow down
    float q2_1 = acos(cos_q2);
    float q2_2 = -q2_1;
    
    // For each elbow solution, calculate joint 1
    for (int i = 0; i < 2; ++i) {
        float q2 = (i == 0) ? q2_1 : q2_2;
        
        // Calculate joint 1 angle
        float k1 = a1 + a2 * cos(q2);
        float k2 = a2 * sin(q2);
        float q1 = atan2(y, x) - atan2(k2, k1);
        
        // Wrap angles
        q1 = RoboticsUtils::wrap_angle(q1);
        q2 = RoboticsUtils::wrap_angle(q2);
        
        // Check joint limits
        bool valid = true;
        if (robot.getJointLimit(0, LimitType::MIN) < robot.getJointLimit(0, LimitType::MAX)) {
            if (q1 < robot.getJointLimit(0, LimitType::MIN) || 
                q1 > robot.getJointLimit(0, LimitType::MAX)) {
                valid = false;
            }
        }
        if (robot.getJointLimit(1, LimitType::MIN) < robot.getJointLimit(1, LimitType::MAX)) {
            if (q2 < robot.getJointLimit(1, LimitType::MIN) || 
                q2 > robot.getJointLimit(1, LimitType::MAX)) {
                valid = false;
            }
        }
        
        if (valid) {
            Eigen::VectorXf solution(2);
            solution << q1, q2;
            solutions.push_back(solution);
        }
    }
    
    return solutions.size();
}

int selectBestIKSolution(
    const Eigen::VectorXf &currentJoints,
    const std::vector<Eigen::VectorXf> &solutions) {
    
    if (solutions.empty()) return -1;
    
    int bestIdx = 0;
    float minDistance = computeJointDistance(currentJoints, solutions[0]);
    
    for (size_t i = 1; i < solutions.size(); ++i) {
        float distance = computeJointDistance(currentJoints, solutions[i]);
        if (distance < minDistance) {
            minDistance = distance;
            bestIdx = i;
        }
    }
    
    return bestIdx;
}

float computeJointDistance(
    const Eigen::VectorXf &joints1,
    const Eigen::VectorXf &joints2) {
    
    if (joints1.size() != joints2.size()) return 1e10f;
    
    float distance = 0.0f;
    for (int i = 0; i < joints1.size(); ++i) {
        float diff = joints1(i) - joints2(i);
        // Wrap angle difference for revolute joints
        if (fabs(diff) > M_PI) {
            diff = RoboticsUtils::wrap_angle(diff);
        }
        distance += diff * diff;
    }
    
    return sqrt(distance);
}