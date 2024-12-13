/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#include "adore_math/eigen.h"

namespace adore
{
namespace math
{


template<typename T>
std::vector<std::vector<T>>
to_vector( const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& eigenMatrix )
{
  int rows = eigenMatrix.rows();
  int cols = eigenMatrix.cols();

  std::vector<std::vector<T>> stdVector( rows, std::vector<T>( cols ) );

  for( int i = 0; i < rows; ++i )
  {
    for( int j = 0; j < cols; ++j )
    {
      stdVector[i][j] = eigenMatrix( i, j );
    }
  }

  return stdVector;
}

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
to_eigen( const std::vector<std::vector<T>>& stdVector )
{
  int rows = static_cast<int>( stdVector.size() );
  int cols = rows > 0 ? static_cast<int>( stdVector[0].size() ) : 0;

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> eigenMatrix( rows, cols );

  for( int i = 0; i < rows; ++i )
  {
    for( int j = 0; j < cols; ++j )
    {
      eigenMatrix( i, j ) = stdVector[i][j];
    }
  }

  return eigenMatrix;
}

template std::vector<std::vector<double>>                      to_vector( const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& );
template Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> to_eigen( const std::vector<std::vector<double>>& );
} // namespace math
} // namespace adore
