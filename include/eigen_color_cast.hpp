#ifndef _CARTAN_EIGEN_COLOR_CAST_HPP_
#define _CARTAN_EIGEN_COLOR_CAST_HPP_

#include <Eigen/Dense>

namespace cartan {

template <typename ScalarOut, typename ScalarIn, int DimOut, int DimIn>
Eigen::Matrix<ScalarOut, DimOut, 1>
eigen_color_cast(const Eigen::Matrix<ScalarIn, DimIn, 1>& in);

#include "eigen_color_cast.ipp"

} // cartan

#endif /* _CARTAN_EIGEN_COLOR_CAST_HPP_ */
