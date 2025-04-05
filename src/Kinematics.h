/**
 * @file Kinematics.h
 * @brief Abstract kinematics interface and implementation classes
 */

 #ifndef KINEMATICS_H
 #define KINEMATICS_H
 
 #include <Arduino.h>
 #include <vector>
 
 /**
  * @brief Abstract class for kinematics calculations
  */
 class Kinematics {
 public:
   virtual ~Kinematics() {}
   
   /**
    * @brief Forward kinematics - Convert joint positions to Cartesian positions
    * @param jointPositions Joint positions
    * @return Cartesian positions
    */
   virtual std::vector<float> forwardKinematics(const std::vector<float>& jointPositions) = 0;
   
   /**
    * @brief Inverse kinematics - Convert Cartesian positions to joint positions
    * @param cartesianPositions Cartesian positions
    * @return Joint positions
    */
   virtual std::vector<float> inverseKinematics(const std::vector<float>& cartesianPositions) = 0;
 };
 
 /**
  * @brief Cartesian kinematics implementation (1:1 mapping)
  */
 class CartesianKinematics : public Kinematics {
 public:
   /**
    * @brief Forward kinematics for Cartesian system (identity mapping)
    * @param jointPositions Joint positions
    * @return Cartesian positions (same as joint)
    */
   std::vector<float> forwardKinematics(const std::vector<float>& jointPositions) override {
     return jointPositions;
   }
   
   /**
    * @brief Inverse kinematics for Cartesian system (identity mapping)
    * @param cartesianPositions Cartesian positions
    * @return Joint positions (same as Cartesian)
    */
   std::vector<float> inverseKinematics(const std::vector<float>& cartesianPositions) override {
     return cartesianPositions;
   }
 };
 
 // TODO: Add other kinematics classes for different machine types
 // e.g., CoreXY, Delta, etc.
 
 #endif // KINEMATICS_H