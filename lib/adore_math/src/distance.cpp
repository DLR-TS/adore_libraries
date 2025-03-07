#include <adore_math/distance.h>

namespace adore
{
namespace math
{
double
distance_point_to_segment( const Point2d& point, const Point2d& seg_start, const Point2d& seg_end )
{
  double seg_dx        = seg_end.x - seg_start.x;
  double seg_dy        = seg_end.y - seg_start.y;
  double seg_length_sq = seg_dx * seg_dx + seg_dy * seg_dy;

  // If the segment is a single point.
  if( seg_length_sq == 0.0 )
  {
    return distance_2d( point, seg_start );
  }

  // Compute the projection parameter 't' of point onto the segment.
  double t = ( ( point.x - seg_start.x ) * seg_dx + ( point.y - seg_start.y ) * seg_dy ) / seg_length_sq;
  t        = std::max( 0.0, std::min( 1.0, t ) );

  // Compute the projection point.
  Point2d proj;
  proj.x = seg_start.x + t * seg_dx;
  proj.y = seg_start.y + t * seg_dy;
  return distance_2d( point, proj );
}

// Free function: Compute the minimum distance between two polygons by checking
// the distance from each vertex of one polygon to each edge of the other.
double
polygon_distance( const Polygon2d& a, const Polygon2d& b )
{
  double min_distance = std::numeric_limits<double>::max();

  // Check vertices of polygon 'a' against each edge of polygon 'b'.
  for( std::size_t i = 0; i < a.points.size(); ++i )
  {
    for( std::size_t j = 0; j < b.points.size(); ++j )
    {
      std::size_t next_j = ( j + 1 ) % b.points.size();
      double      dist   = distance_point_to_segment( a.points[i], b.points[j], b.points[next_j] );
      if( dist < min_distance )
      {
        min_distance = dist;
      }
    }
  }

  // Check vertices of polygon 'b' against each edge of polygon 'a'.
  for( std::size_t i = 0; i < b.points.size(); ++i )
  {
    for( std::size_t j = 0; j < a.points.size(); ++j )
    {
      std::size_t next_j = ( j + 1 ) % a.points.size();
      double      dist   = distance_point_to_segment( b.points[i], a.points[j], a.points[next_j] );
      if( dist < min_distance )
      {
        min_distance = dist;
      }
    }
  }

  return min_distance;
}


} // namespace math
} // namespace adore