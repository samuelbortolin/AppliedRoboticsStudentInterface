#include "utils.hpp"
#include <bits/stdc++.h>


/*!
* Run the UCS algorithm in order to obtain the optimal cost to the target for each node.
* @param[in]  adjacency_matrix            The adjacency matrix of the roadmap.
* @param[in]  target                      The index of the target node.
*/
std::vector<float> ucs(std::vector< std::vector<float> > adjacency_matrix, int target);


/*!
* Coordinate the motion between the robots and finds the feasible path for all robots (i.e., no collisions with obstacles or other robots).
* @param[in] optimal_cost		The optimal cost to the target for each node.
* @param[in] adjacency_matrix		The adjancy matrix defining the roadmap.
* @param[in] initial_nodes		The nodes from which the robots start.
* @param[in] nodes			The nodes of the roadmap.
*/
std::vector<std::vector<int>> find_optimal_paths(std::vector<float> optimal_cost, std::vector<std::vector<float>> adjacency_matrix, std::vector<int> initial_nodes, std::vector<Point> nodes);

