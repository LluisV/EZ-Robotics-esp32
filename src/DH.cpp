#include "DH.h"
#include "RoboticsUtils.h"
#include <cmath>
#include <cassert>


DH::DH(int numberOfJoints, RobotType robotType)
    : numberOfJoints(numberOfJoints), robotType(robotType), 
      cachingEnabled(false), cacheValid(false) {
    // Initialize joints with default values
    joints.resize(numberOfJoints);
    cachedTransforms.resize(numberOfJoints);
}

void DH::setJointParameters(
    int jointIndex, 
    double theta, 
    double d, 
    double a, 
    double alpha, 
    JointType jointType) {
    
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    DHJoint& joint = joints[jointIndex];
    joint.theta = theta;
    joint.d = d;
    joint.a = a;
    joint.alpha = alpha;
    joint.type = jointType;
    
    invalidateCache();
}


bool DH::setJointPosition(int jointIndex, double position) {
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    DHJoint& joint = joints[jointIndex];
    
    // Check if joint limits are defined and position is within limits
    if (joint.minLimit < joint.maxLimit) {
        double adjustedPosition = position;
        
        // Wrap angle for revolute joints
        if (joint.type == JointType::REVOLUTE) {
            adjustedPosition = RoboticsUtils::wrap_angle(position);
        }
        
        if (adjustedPosition < joint.minLimit || adjustedPosition > joint.maxLimit) {
            return false;
        }
    }
    
    // Update the appropriate parameter based on joint type
    if (joint.type == JointType::REVOLUTE) {
        joint.theta = RoboticsUtils::wrap_angle(position);
    } else {
        joint.d = position;
    }
    
    invalidateCache();
    return true;
}


bool DH::setJointPositions(const Eigen::VectorXf& positions) {
    assert(positions.size() == numberOfJoints);
    
    // First check if all positions are within limits
    if (!areJointPositionsWithinLimits(positions)) {
        return false;
    }
    
    // Apply all positions
    for (int i = 0; i < numberOfJoints; ++i) {
        if (joints[i].type == JointType::REVOLUTE) {
            joints[i].theta = RoboticsUtils::wrap_angle(positions(i));
        } else {
            joints[i].d = positions(i);
        }
    }
    
    invalidateCache();
    return true;
}


double DH::getJointPosition(int jointIndex) const {
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    const DHJoint& joint = joints[jointIndex];
    return (joint.type == JointType::REVOLUTE) ? joint.theta : joint.d;
}


Eigen::VectorXf DH::getJointPositions() const {
    Eigen::VectorXf positions(numberOfJoints);
    
    for (int i = 0; i < numberOfJoints; ++i) {
        positions(i) = getJointPosition(i);
    }
    
    return positions;
}


void DH::setJointLimits(int jointIndex, double minValue, double maxValue) {
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    joints[jointIndex].minLimit = minValue;
    joints[jointIndex].maxLimit = maxValue;
}


double DH::getJointLimit(int jointIndex, LimitType limitType) const {
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    return (limitType == LimitType::MIN) ? 
           joints[jointIndex].minLimit : 
           joints[jointIndex].maxLimit;
}


bool DH::areJointPositionsWithinLimits(const Eigen::VectorXf& positions) const {
    // Ensure the number of positions matches the number of joints
    assert(positions.size() == numberOfJoints);
    
    // Define a small tolerance value to account for precision errors
    constexpr double epsilon = 1e-6; 

    // Iterate through each joint to check its position
    for (int i = 0; i < numberOfJoints; ++i) {
        const DHJoint& joint = joints[i];

        // If the joint has valid limits (min < max)
        if (joint.minLimit < joint.maxLimit) {
            double value = positions(i);

            // If the joint is revolute, wrap the angle to ensure it stays within valid range
            if (joint.type == JointType::REVOLUTE) {
                value = RoboticsUtils::wrap_angle(value);
            }

             // Check if the joint value is outside the defined limits with tolerance
            if (value < joint.minLimit - epsilon || value > joint.maxLimit + epsilon) {
                return false; // Position is out of bounds
            }
        }
    }

    // All positions are within the limits
    return true;
}


Eigen::Matrix4f DH::getJointTransform(int jointIndex) const {
    if (cachingEnabled) {
        if (!cacheValid) {
            updateCache();
        }
        return cachedTransforms[jointIndex];
    }
    return getCachedJointTransform(jointIndex);
}


Eigen::Matrix4f DH::getCachedJointTransform(int jointIndex) const {
    // Ensure the joint index is valid (within the range of joints)
    assert(jointIndex >= 0 && jointIndex < numberOfJoints);
    
    const DHJoint& joint = joints[jointIndex];
    
    // Calculate trigonometric values for the DH parameters
    double ct = cos(joint.theta);  // cos(theta)
    double st = sin(joint.theta);  // sin(theta)
    double ca = cos(joint.alpha);  // cos(alpha)
    double sa = sin(joint.alpha);  // sin(alpha)
    double d = joint.d;            // d parameter
    double a = joint.a;            // a parameter
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
    // Apply the standard Denavit-Hartenberg convention to build the transformation matrix
    transform << ct, -st * ca, st * sa, a * ct,
                    st, ct * ca, -ct * sa, a * st,
                    0, sa, ca, d,
                    0, 0, 0, 1;
                     
    return transform;
}


Eigen::Matrix4f DH::getTransformUpToJoint(int endJointIndex) const {
    // If endJointIndex is negative, use all joints
    if (endJointIndex < 0) endJointIndex = numberOfJoints;
    
    // Verify endJointIndex is valid
    assert(endJointIndex <= numberOfJoints);
    
    if (cachingEnabled) {
        if (!cacheValid) {
            updateCache();
        }
        if (endJointIndex == numberOfJoints) {
            return cachedEndEffectorTransform;
        }
    }
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    
    // Multiply the transformation matrices of each joint up to the specified endJointIndex
    for (int i = 0; i < endJointIndex; ++i) {
        if (cachingEnabled && cacheValid) {
            transform *= cachedTransforms[i];
        } else {
            transform *= getCachedJointTransform(i);
        }
    }
    
    return transform;
}


Eigen::VectorXf DH::getEndEffectorPose() const {
    Eigen::Matrix4f transform = getTransformUpToJoint(-1);
    Eigen::Matrix3f rotationMatrix = transform.block<3, 3>(0, 0);
    
    // Extract position
    Eigen::Vector3f position = transform.block<3, 1>(0, 3);
    
    // Calculate Euler angles (XYZ convention)
    float roll, pitch, yaw;
    
    // Extract yaw (around z-axis)
    yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    
    // Extract pitch (around y-axis)
    pitch = atan2(-rotationMatrix(2, 0), 
                  sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + 
                       rotationMatrix(2, 2) * rotationMatrix(2, 2)));
    
    // Extract roll (around x-axis)
    roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));

    // Create the pose vector
    Eigen::VectorXf pose(6);
    pose << position, roll, pitch, yaw;
    
    return pose;
}


Eigen::VectorXf DH::getEndEffectorPoseQuaternion() const {
    Eigen::Matrix4f transform = getTransformUpToJoint(-1);
    Eigen::Matrix3f rotationMatrix = transform.block<3, 3>(0, 0);
    
    // Extract position
    Eigen::Vector3f position = transform.block<3, 1>(0, 3);
    
    // Convert rotation matrix to quaternion
    Eigen::Quaternionf q(rotationMatrix);
    
    // Create the pose vector with quaternion
    Eigen::VectorXf pose(7);
    pose << position, q.w(), q.x(), q.y(), q.z();
    
    return pose;
}


int DH::getNumberOfJoints() const {
    return numberOfJoints;
}


Eigen::Matrix<float, 6, Eigen::Dynamic> DH::computeJacobian() const {

    // Initialize the Jacobian matrix as a 6x(numberOfJoints) zero matrix
    Eigen::Matrix<float, 6, Eigen::Dynamic> jacobian =
        Eigen::Matrix<float, 6, Eigen::Dynamic>::Zero(6, numberOfJoints);
    
    // Get the full transformation matrix for the end effector
    Eigen::Matrix4f T_end = getTransformUpToJoint(-1);
    Eigen::Vector3f p_end = T_end.block<3,1>(0, 3); // End effector position

    // Iterate over each joint to calculate its contribution to the Jacobian
    for (int i = 0; i < numberOfJoints; ++i) {
        Eigen::Matrix4f T_i = getTransformUpToJoint(i);
        Eigen::Vector3f z_i = T_i.block<3, 1>(0, 2);  // Z axis of the i-1 frame
        Eigen::Vector3f p_i = T_i.block<3, 1>(0, 3);  // Origin of the i-1 frame
        
        if (joints[i].type == JointType::REVOLUTE) {
             // For revolute (rotational) joints
            jacobian.block<3, 1>(0, i) = z_i.cross(p_end - p_i); // Linear velocity contribution
            jacobian.block<3, 1>(3, i) = z_i; // Angular velocity contribution
        } else {
            // For prismatic (translational) joints
            jacobian.block<3, 1>(0, i) = z_i; // Linear velocity contribution
            jacobian.block<3, 1>(3, i) = Eigen::Vector3f::Zero();  // No angular velocity contribution
        }
    }

    // Return the computed Jacobian matrix
    return jacobian;
}


Eigen::VectorXf DH::computeEndEffectorVelocity(
    const Eigen::VectorXf& jointVelocities) const {
    
    assert(jointVelocities.size() == numberOfJoints);
    
    // Compute the Jacobian matrix at the current configuration
    Eigen::Matrix<float, 6, Eigen::Dynamic> jacobian = computeJacobian();
    
    // Multiply the Jacobian by the joint velocities to get the end-effector velocity
    return jacobian * jointVelocities;
}


float DH::computeManipulability() const {
    // Special case for Cartesian robots - they don't have singularities
    if (robotType == RobotType::CARTESIAN) {
        // For Cartesian robots, return a constant high value
        // indicating no kinematic singularities
        return 1.0f;
    }
    
    // Compute the Jacobian
    Eigen::MatrixXf J = computeJacobian();
    
    // Compute J * J^T
    Eigen::MatrixXf JJT = J * J.transpose();
    
    // Compute the determinant
    float det = JJT.determinant();
    
    // Return sqrt of determinant (manipulability measure)
    return (det > 0) ? sqrt(det) : 0.0f;
}


bool DH::isNearSingularity(float threshold) const {
    return computeManipulability() < threshold;
}


bool DH::isPositionReachable(const Eigen::Vector3f& position) const {
    float distance = position.norm();
    float maxReach = getMaxReach();
    float minReach = getMinReach();
    
    // Basic reachability check
    if (distance > maxReach || distance < minReach) {
        return false;
    }
    
    // Additional checks for specific robot types
    if (robotType == RobotType::CARTESIAN) {
        // For Cartesian robots, check if position is within the box limits
        for (int i = 0; i < 3 && i < numberOfJoints; ++i) {
            const DHJoint& joint = joints[i];
            if (joint.type == JointType::PRISMATIC && joint.minLimit < joint.maxLimit) {
                // Map position coordinates to joint limits based on DH parameters
                float coord = 0;
                if (i == 0) coord = position.z();      // First prismatic usually Z
                else if (i == 1) coord = position.x();  // Second prismatic usually X
                else if (i == 2) coord = position.y();  // Third prismatic usually Y
                
                if (coord < joint.minLimit || coord > joint.maxLimit) {
                    return false;
                }
            }
        }
    }
    
    return true;
}


float DH::getMaxReach() const {
    float maxReach = 0.0f;
    
    // Special handling for Cartesian robots
    if (robotType == RobotType::CARTESIAN) {
        // For Cartesian robots, max reach is the diagonal of the workspace
        float x_max = 0, y_max = 0, z_max = 0;
        
        // Assuming standard Cartesian configuration (Z, X, Y)
        for (int i = 0; i < numberOfJoints && i < 3; ++i) {
            const DHJoint& joint = joints[i];
            if (joint.type == JointType::PRISMATIC && joint.maxLimit > joint.minLimit) {
                float range = joint.maxLimit - joint.minLimit;
                if (i == 0) z_max = joint.maxLimit;  // Z axis
                else if (i == 1) x_max = joint.maxLimit;  // X axis
                else if (i == 2) y_max = joint.maxLimit;  // Y axis
            }
        }
        
        // Return the diagonal distance from origin to far corner
        return sqrt(x_max*x_max + y_max*y_max + z_max*z_max);
    }
    
    // Original calculation for other robot types
    // Sum all link lengths and maximum prismatic extensions
    for (int i = 0; i < numberOfJoints; ++i) {
        const DHJoint& joint = joints[i];
        maxReach += joint.a;  // Add link length
        
        // Add maximum prismatic extension if applicable
        if (joint.type == JointType::PRISMATIC && joint.maxLimit > joint.minLimit) {
            maxReach += joint.maxLimit;
        } else if (joint.type == JointType::PRISMATIC) {
            maxReach += abs(joint.d);
        }
    }
    
    return maxReach;
}


float DH::getMinReach() const {
    float minReach = 0.0f;
    
    // For articulated robots, minimum reach can be the difference of link lengths
    if (robotType == RobotType::ARTICULATED_2R || robotType == RobotType::SCARA) {
        float sum = 0.0f;
        float max_link = 0.0f;
        
        for (int i = 0; i < numberOfJoints; ++i) {
            const DHJoint& joint = joints[i];
            if (joint.type == JointType::REVOLUTE) {
                sum += joint.a;
                max_link = fmax(max_link, joint.a);
            }
        }
        
        minReach = fmax(0.0f, 2 * max_link - sum);
    }
    
    return minReach;
}


void DH::enableTransformCaching(bool enable) {
    cachingEnabled = enable;
    if (!enable) {
        cacheValid = false;
    }
}


void DH::invalidateCache() {
    cacheValid = false;
}


void DH::updateCache() const {
    if (!cachingEnabled) return;
    
    // Cache individual joint transforms
    for (int i = 0; i < numberOfJoints; ++i) {
        cachedTransforms[i] = getCachedJointTransform(i);
    }
    
    // Cache end effector transform
    cachedEndEffectorTransform = Eigen::Matrix4f::Identity();
    for (int i = 0; i < numberOfJoints; ++i) {
        cachedEndEffectorTransform *= cachedTransforms[i];
    }
    
    cacheValid = true;
}