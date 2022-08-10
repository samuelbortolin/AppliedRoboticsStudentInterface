#include "utils.hpp"
#include "intersection.hpp"


/*!
* Sort vertices of polygons.
* @param[in]  obstacles           The obstacles.
*/
std::vector<Point> sort_vertices(std::vector< std::vector<Point> > obstacles);


/*!
* Check if there is a intersection between an arc and a segment.
* @param[in]  boundary            The boundary of the arena.
* @param[in]  sorted_vertices     The sorted vertices.
* @param[in]  obstacles           The obstacles.
*/
std::vector< std::vector<Point> > create_segments_vertical_decomposition(std::vector<Point> boundary, std::vector<Point> sorted_vertices, std::vector<Polygon> obstacles);

