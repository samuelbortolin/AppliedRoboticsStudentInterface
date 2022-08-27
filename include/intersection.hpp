#include "utils.hpp"


/*!
* Normalize an angle in range [0,2*pi).
* @param[in]  angle		The angle to normalize.
*/
float mod2Pi(float angle);


/*!
* Check if there is a intersection between an arc and a segment.
* @param[in]  a		The center of the circle.
* @param[in]  r		The radius of the circe.
* @param[in]  s		The starting angle of the arc.
* @param[in]  e		The ending angle of the arc.
* @param[in]  s2	The starting point of the segment.
* @param[in]  e2	The ending point of the segment.
*/
bool intersection_arc_segment(Point a, double r, double s, double e, Point s2, Point e2);


/*!
* Check if there is a intersection between two arcs.
* @param[in]  a1	The center of the first circe.
* @param[in]  r1	The radius of the first circe.
* @param[in]  s1	The starting angle of the first arc.
* @param[in]  e1	The ending angle of the first arc.
* @param[in]  a2	The center of the second circe.
* @param[in]  r2	The radius of the second circe.
* @param[in]  s2	The starting angle of the second arc.
* @param[in]  e2	The ending angle of the second arc.
*/
bool intersection_arc_arc(Point a1, double r1, double s1, double e1, Point a2, double r2, double s2, double e2);


/*!
* Check if there is a intersection between two segments.
* @param[in]  s1	The starting point of the first segment.
* @param[in]  e1	The ending point of the first segment.
* @param[in]  s2	The starting point of the second segment.
* @param[in]  e2	The ending point of the second segment.
*/
bool intersection_segment_segment(Point s1, Point e1, Point s2, Point e2);


/*!
* Check if there is a intersection between two segments and return a point of intersection.
* @param[in]  s1	The starting point of the first segment.
* @param[in]  e1	The ending point of the first segment.
* @param[in]  s2	The starting point of the second segment.
* @param[in]  e2	The ending point of the second segment.
*/
Point get_intersection_point_segment_segment(Point s1, Point e1, Point s2, Point e2);


/*!
* Check if there is a intersection between a segment and the obstacles.
* @param[in]  p1		The starting point of the first segment.
* @param[in]  p2		The ending point of the first segment.
* @param[in]  s2		The starting point of the second segment.
* @param[in]  obstacles		The obstacles.
*/
bool get_intersection_segment_obstacles(Point s1, Point e1, std::vector<Polygon> obstacles);

