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
#ifndef KALMAN_MEASUREMENTMODEL_HPP_
#define KALMAN_MEASUREMENTMODEL_HPP_

#include "Concepts.hpp"
#include "StandardBase.hpp"

namespace Kalman {
/**
 * @brief Abstract base class of all measurement models
 *
 * @param StateType The vector-type of the system state (usually some type
 * derived from Kalman::Vector)
 * @param MeasurementType The vector-type of the measurement (usually some type
 * derived from Kalman::Vector)
 * @param CovarianceBase The class template used for covariance storage (must be
 * either StandardBase or SquareRootBase)
 */
template <StateVector StateType, MeasurementVector MeasurementType,
          template <class> class CovarianceBase = StandardBase>
requires SameScalar<StateType, MeasurementType> class MeasurementModel
    : public CovarianceBase<MeasurementType> {
public:
  //! System state type
  using State = StateType;

  //! Measurement vector type
  using Measurement = MeasurementType;

public:
  /**
   * Measurement Model Function h
   *
   * Predicts the estimated measurement value given the current state estimate x
   */
  virtual Measurement h(const State &x) const = 0;

protected:
  MeasurementModel() = default;
  virtual ~MeasurementModel() = default;
};
} // namespace Kalman

#endif
