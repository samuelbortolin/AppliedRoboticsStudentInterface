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


std::vector<std::vector<int>> find_optimal_paths(std::vector<float> optimal_cost, std::vector<std::vector<float>> adjacency_matrix, std::vector<int> initial_nodes){
	int arrived_robots = 0;
	int target = adjacency_matrix.size()-1;
	std::vector<std::vector<int>> optimal_paths = {};
	for(int initial_node : initial_nodes){
		optimal_paths.push_back({initial_node});
	}
	while(arrived_robots < initial_nodes.size()){
		
		for(int i=0; i<initial_nodes.size(); i++){
			int current_node = optimal_paths[i].back();
			if(current_node == target){
				continue;
			}
			float smallest_cost = optimal_cost[current_node];
			int optimal_node = current_node;
			for(int j=0; j<adjacency_matrix.size(); j++){
				if(adjacency_matrix[current_node][j] != 0.0){
					if(smallest_cost > optimal_cost[j]){
						smallest_cost = optimal_cost[j];
						optimal_node = j;
					}
				}
			}
			optimal_paths[i].push_back(optimal_node);
			if(optimal_node == target){  // TODO: avoid infinite loop if we are no able to reach the target
				arrived_robots++;
			}
		}
	}
	
	return optimal_paths;
}

