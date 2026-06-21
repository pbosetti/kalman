// The MIT License (MIT)
//
// Copyright (c) 2015 Markus Herb
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef KALMAN_CONCEPTS_HPP_
#define KALMAN_CONCEPTS_HPP_

//! \file
//! \brief C++20 concepts describing the vector types accepted by the filters.
//!
//! These replace the hand-written `static_assert`s of the original library and
//! give substantially clearer diagnostics: instantiating a filter with an
//! unsuitable state/measurement/control type now fails at the constraint with
//! a readable message instead of deep inside a template expansion.

#include <concepts>

#include "Matrix.hpp"

namespace Kalman {

/**
 * @brief Satisfied by Eigen-style column vectors.
 *
 * Requires a `Scalar` type and compile-time row/column counts, with exactly
 * one column.
 */
template <typename T> concept ColumnVector = requires {
  typename T::Scalar;
  { static_cast<int>(T::RowsAtCompileTime) } -> std::convertible_to<int>;
  { static_cast<int>(T::ColsAtCompileTime) } -> std::convertible_to<int>;
}
&&(static_cast<int>(T::ColsAtCompileTime) == 1);

/**
 * @brief A system state: a non-empty column vector.
 */
template <typename T>
concept StateVector =
    ColumnVector<T> && (static_cast<int>(T::RowsAtCompileTime) > 0);

/**
 * @brief A measurement: a non-empty column vector.
 */
template <typename T>
concept MeasurementVector =
    ColumnVector<T> && (static_cast<int>(T::RowsAtCompileTime) > 0);

/**
 * @brief A control input: a column vector that may be empty (0 rows).
 */
template <typename T>
concept ControlVector =
    ColumnVector<T> && (static_cast<int>(T::RowsAtCompileTime) >= 0);

/**
 * @brief Satisfied when two vector types use the same scalar type.
 */
template <typename A, typename B>
concept SameScalar = std::same_as<typename A::Scalar, typename B::Scalar>;

} // namespace Kalman

#endif
