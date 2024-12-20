#include <cmath>

#include <iostream>

namespace adore
{
namespace math
{


struct Box3d
{
  // Dimensions
  double length;
  double width;
  double height;

  Box3d() {};
  Box3d( double length, double width, double height ) :
    length( length ),
    width( width ),
    height( height ) {};
};
} // namespace math
} // namespace adore