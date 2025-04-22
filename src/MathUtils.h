/**
 * @file MathUtils.h
 * @brief Mathematical utility functions for motion planning and vector operations
 */

 #ifndef MATH_UTILS_H
 #define MATH_UTILS_H
 
 #include <vector>
 #include <cmath>
 
 /**
  * @brief Utility class for mathematical operations
  */
 class MathUtils {
 public:
     /**
      * @brief Normalize a vector to unit length
      * @param vec Vector to normalize
      * @return Normalized vector with unit length
      */
     static std::vector<float> normalizeVector(const std::vector<float>& vec);
     
     /**
      * @brief Calculate the dot product of two vectors
      * @param vec1 First vector
      * @param vec2 Second vector
      * @return The dot product value
      */
     static float dotProductVectors(const std::vector<float>& vec1, const std::vector<float>& vec2);
     
     /**
      * @brief Calculate the angle between two vectors in radians
      * @param vec1 First vector
      * @param vec2 Second vector
      * @return Angle in radians
      */
     static float angleBetweenVectors(const std::vector<float>& vec1, const std::vector<float>& vec2);
     

 };
 
 #endif // MATH_UTILS_H