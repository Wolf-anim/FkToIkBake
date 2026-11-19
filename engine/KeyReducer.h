// FkToIkBake - Matching and Baking Engine
// KeyReducer.h : optional key reduction for baked channels.
//
//  * Constant channels are always collapsed to their first and last key
//    (lossless with any tangent type).
//  * With tolerance > 0, Ramer-Douglas-Peucker on each channel against
//    LINEAR interpolation: every removed sample is reproduced within
//    `tolerance` by the remaining keys, provided they use linear tangents.
//    The Maya layer forces linear tangents whenever reduction is active, so
//    the error bound is a guarantee, not an estimate.
#pragma once

#include <cstddef>
#include <vector>

namespace fk2ik {

// Returns sorted indices of samples to keep.  Always includes first & last.
std::vector<size_t> reduceKeys(const std::vector<double>& times, const std::vector<double>& values,
                               double tolerance);

bool isConstant(const std::vector<double>& values, double eps = 1e-9);

}  // namespace fk2ik
