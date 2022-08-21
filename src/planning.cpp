#include "planning.hpp"

// TODO: add the BFS algorithm
// std::vector< std::vector<int> > bfs(std::vector< std::vector<int> > adjacency_matrix, int source, int target);


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
	return best_cost_so_far;
}


/*std::vector< std::vector<int> > bfs(std::vector< std::vector<int> > adjacency_matrix, int source, int target, std::vector<Point> centroids){
	std::vector<bool> explored;
	int path_cost[adjacency_matrix.size()];
	// node, cost
	std::vector<std::pair<int, int>> parents;
	std::vector< std::vector<int> > paths;
	// cost, node
	std::priority_queue<std::pair<int, int>> queue;
	
	for(int i=0; i < adjacency_matrix.size(); i++){
		explored.push_back(false);
		path_cost[i] = 1
		parents[i] = make_pair(-1, 0);
	}
	
	queue.push(std::make_pair(0, source));
	while (queue.empty()) {
		int current_edge = queue.top().second;
		queue.pop();
		explored[current_edge] = true;
		if(current_edge == target){
			break;
		}
		for (int child_node=0; child_node<adjacency_matrix.size(); child_node++){
			if(adjacency_matrix[child_node][current_edge] != 0 && !explored[child_node]){
				int current_cost = adjacency_matrix[child_node][current_edge] * (-1);
				path_cost[child_node] = path_cost[current_node] - current_cost;
				if(parents[child_node].first < 0){
					parents[child_node] = make_pair(current_node, path_cost[child_node]);		
				} else if(parents[child_node].second < path_cost[child_node]){
					parents[child_node] = make_pair(current_node, path_cost[child_node]);
				}
				
				queue.push(std::make_pair(path_cost[child_node], child_node));
			}
		}
	}
}*/


