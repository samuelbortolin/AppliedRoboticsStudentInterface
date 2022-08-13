#include "utils.hpp"
#include "clipper/clipper.hpp"


/*!
* Add an offset to the obsticales in the arena to account for the size of the robot
* @param[out] obstacle_list  The list of obstacle polygon
* @param[in]  offset_value   The offset to be added to the obstacles
*/
std::vector<Polygon> add_offset_to_obstacles(const std::vector<Polygon>& obstacle_list, float offset_value);


// TODO: add a method to merge overlapping obstacles


/*!
* Add an offset to the borders of the arena to account for the size of the robot
* @param[in]  borders        The border of the arena
* @param[in]  offset_value   The offset to be added to the obstacles
*/
Polygon add_offset_to_borders(const Polygon& borders, float offset_value);

