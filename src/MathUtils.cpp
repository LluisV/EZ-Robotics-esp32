/**
 * @file MathUtils.cpp
 * @brief Implementation of mathematical utility functions
 */

#include "MathUtils.h"
#include <algorithm>

std::vector<float> MathUtils::normalizeVector(const std::vector<float> &vec)
{
    std::vector<float> result = vec;
    float magnitude = 0.0f;

    // Calculate magnitude
    for (size_t i = 0; i < vec.size(); i++)
    {
        magnitude += vec[i] * vec[i];
    }
    magnitude = sqrt(magnitude);

    // Avoid division by zero
    if (magnitude > 0.000001f)
    {
        for (size_t i = 0; i < result.size(); i++)
        {
            result[i] /= magnitude;
        }
    }

    return result;
}

float MathUtils::dotProductVectors(const std::vector<float> &vec1, const std::vector<float> &vec2)
{
    float dotProduct = 0.0f;
    size_t minSize = std::min(vec1.size(), vec2.size());

    for (size_t i = 0; i < minSize; i++)
    {
        dotProduct += vec1[i] * vec2[i];
    }

    return dotProduct;
}

float MathUtils::angleBetweenVectors(const std::vector<float> &vec1, const std::vector<float> &vec2)
{
    // Get normalized vectors
    std::vector<float> normVec1 = normalizeVector(vec1);
    std::vector<float> normVec2 = normalizeVector(vec2);

    // Calculate dot product
    float dot = dotProductVectors(normVec1, normVec2);

    // Clamp to [-1, 1] range to avoid floating point errors
    dot = std::max(-1.0f, std::min(1.0f, dot));

    // Return angle in radians
    return acos(dot);
}

