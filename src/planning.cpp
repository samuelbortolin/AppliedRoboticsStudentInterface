#include "planning.hpp"


std::vector<float> ucs(std::vector< std::vector<float> > adjacency_matrix, int target){
	std::vector<bool> explored;
	// The current optimal cost
	std::vector<float> best_cost_so_far;
	// cost, node
	std::priority_queue<std::pair<float, int>> queue;

	for (int i=0; i < adjacency_matrix.size(); i++){
		explored.push_back(false);
		best_cost_so_far.push_back(1.0);
	}

	queue.push(std::make_pair(0.0, target));
	best_cost_so_far[target] = 0.0;

	while (!queue.empty()){
		int current_node = queue.top().second;
		// priority_queue doesn't provide a replace method so nodes could be present multiple times inside the queue.
		if (explored[current_node]){
			queue.pop();
			continue;
		}
		queue.pop();
		explored[current_node] = true;
		for (int i=0; i<adjacency_matrix.size(); i++){
			if (adjacency_matrix[i][current_node] != 0.0 && !explored[i]){
				int child_node = i;
				float cost_to_arrive_from_current_path;
				if (best_cost_so_far[current_node] > 0.0){
					// Priority queue is ordering in descending order, so to have it ordered in ascending order as UCS request it the costs are multiplied by -1
					cost_to_arrive_from_current_path = (-1.0) * adjacency_matrix[current_node][child_node];
				} else {
					cost_to_arrive_from_current_path = best_cost_so_far[current_node] - adjacency_matrix[current_node][child_node];
				}
				if ((!explored[child_node] && best_cost_so_far[child_node] > 0.0) || best_cost_so_far[child_node] < cost_to_arrive_from_current_path){
					best_cost_so_far[child_node] = cost_to_arrive_from_current_path;
					queue.push(std::make_pair(cost_to_arrive_from_current_path, child_node));
				}
			}
		}
	}
	
	for(int i=0; i<best_cost_so_far.size(); i++){
		best_cost_so_far[i] = best_cost_so_far[i] * (-1.0);
	}
	return best_cost_so_far;
}


std::vector<std::vector<int>> find_optimal_paths(std::vector<float> optimal_cost, std::vector<Point> nodes, std::vector<std::vector<float>> adjacency_matrix, std::vector<int> initial_nodes, int target_node, float offset_value){
	int arrived_robots = 0;
	std::vector<std::vector<int>> optimal_paths = {};
	for (int initial_node : initial_nodes){
		optimal_paths.push_back({initial_node});
		if (initial_node == target_node){
			arrived_robots++;
		}
	}
	while (arrived_robots < initial_nodes.size()){
		std::vector<int> inaccessible_nodes = {};
		for (int i=0; i<initial_nodes.size(); i++){
			int current_node = optimal_paths[i].back();
			if (current_node == target_node){
				continue;
			}
			float smallest_cost = optimal_cost[current_node];
			int optimal_node = current_node;
			for (int j=0; j<adjacency_matrix.size(); j++){
				// If the node is neighbour and, it's free or it is the gate and the reciprocal distance to the target w.r.t. to other robots is bigger than offset_value.
				bool target_free = true;
				for (int k=0; k<i; k++){
					if (optimal_paths[k].size() > optimal_paths[i].size() && optimal_cost[optimal_paths[k][optimal_paths[i].size()]] < offset_value && std::abs(optimal_cost[optimal_paths[k][optimal_paths[i].size()]] - optimal_cost[optimal_paths[i].back()]) < offset_value){
						target_free = false;
					}
				}
				if (adjacency_matrix[current_node][j] != 0.0 && (std::find(inaccessible_nodes.begin(), inaccessible_nodes.end(), j) == inaccessible_nodes.end() || (j==target_node && target_free))){
					// Check if it has a smaller cost.
					if (smallest_cost > optimal_cost[j]){
						smallest_cost = optimal_cost[j];
						optimal_node = j;
					}
				}
			}
			optimal_paths[i].push_back(optimal_node);
			if (optimal_node == target_node){
				arrived_robots++;
			}

			// Add to the inaccessible nodes list the nodes that are close to the next robot position
			for (int j=0; j<nodes.size(); j++){
				float eucl_distance = sqrt(pow(nodes[optimal_node].x - nodes[j].x, 2) + pow(nodes[optimal_node].y - nodes[j].y, 2));
				if (eucl_distance < offset_value){
					inaccessible_nodes.push_back(j);
				}
			}
		}
	}
	return optimal_paths;
}

