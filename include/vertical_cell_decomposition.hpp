#include <tuple>
#include <algorithm>
#include "utils.hpp"
#include "intersection.hpp"


/*!
* Sort vertices of polygons.
* @param[in]  obstacles		The obstacles.
*/
std::vector< std::tuple<Point, int> > sort_vertices(std::vector<Polygon> obstacles);


/*!
* Get the vertical segments generated by the VCD algorithm.
* @param[in]  sorted_vertices		The sorted vertices.
* @param[in]  obstacles			The obstacles.
* @param[in]  lower_limit		The lower limit of the arena (y coordinate).
* @param[in]  upper_limit		The upper limit of the arena (y coordinate).
*/
std::vector< std::vector<Point> > create_segments_vertical_decomposition(std::vector <std::tuple<Point, int> > sorted_vertices, std::vector<Polygon> obstacles, float lower_limit, float upper_limit);


/*!
* Find the cells generated from the VCD algorithm.
* @param[in]  boundary			The boundary of the arena.
* @param[in]  sorted_vertices		The sorted vertices.
* @param[in]  obstacles			The obstacles.
*/
std::vector<Polygon> find_cells(std::vector<Point> boundary, std::vector <std::tuple<Point, int> > sorted_vertices, std::vector<Polygon> obstacles);


/*!
* Get the cell centroid.
* @param[in]  cell            The cell for which return the cenntroid.
*/
Point get_cell_centroid(Polygon cell);


/*!
* Create the roadmap returning the list of nodes and the adjacency matrix describing how they are connected to each other and how much they are distant if connected.
* @param[in]  cells		The cells generated from the VCD algorithm.
* @param[in]  obstacles		The obstacles.
* @param[in]  gate_list		The list of gates.
* @param[in]  x			The list of x coordinates of the robots.
* @param[in]  y			The list of y coordinates of the robots.
*/
std::tuple< std::vector<Point>, std::vector< std::vector<float> > > create_roadmap(std::vector<Polygon> cells, std::vector<Polygon> obstacles, const std::vector<Polygon>& gate_list, const std::vector<float> x, const std::vector<float> y);

