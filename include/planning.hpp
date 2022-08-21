#include "utils.hpp"
#include <bits/stdc++.h>

// TODO: define a method to coordinate the motion between the robots and check if the path found is ok for all robots (i.e., no collisions with obstacles or other robots)


/*!
* Run the BFS algorithm in order to obtain all the possible paths that leads to the target. // TODO: probably we just need the first that ensures that the paths is ok for all robots
* @param[in]  adjacency_matrix            The adjacency matrix of the roadmap.
* @param[in]  target                      The index of the target node.
*/
std::vector<float> ucs(std::vector< std::vector<float> > adjacency_matrix, int target);

