#include "vertical_cell_decomposition.hpp"


std::vector<Point> sort_vertices(std::vector< std::vector<Point> > obstacles){
	float vertices_num;
	for (int i = 0; i < obstacles.size(); i++){
		for (const auto &point : obstacles[i]){
			vertices_num += 1;
		}
	}

	std::vector<Point> sorted_vertices;
	int add_to_list;

	for (int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--){
		sorted_vertices.push_back(Point(-1,-1));
	}

	for (int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--){
		for (int obj = 0; obj < obstacles.size(); obj++){
			for (int vertex = 0; vertex < obstacles[obj].size(); vertex++){
				add_to_list = 0;
				if (obstacles[obj][vertex].x > sorted_vertices[curr_vertex].x){
					if (curr_vertex == vertices_num - 1 ||
						obstacles[obj][vertex].x < sorted_vertices[curr_vertex + 1].x ||
						obstacles[obj][vertex].x == sorted_vertices[curr_vertex + 1].x){
						add_to_list = 1;
					}

					for (int vert = 0; vert < sorted_vertices.size(); vert ++){
						if (sorted_vertices[vert].x == obstacles[obj][vertex].x && sorted_vertices[vert].y == obstacles[obj][vertex].y){
							add_to_list = 0;
						}
					}
				}
				if (add_to_list == 1){
					sorted_vertices[curr_vertex].x = obstacles[obj][vertex].x;
					sorted_vertices[curr_vertex].y = obstacles[obj][vertex].y;
				}
			}             
		} 
	}
	return sorted_vertices;
}


std::vector< std::vector<Point> > create_segments_vertical_decomposition(std::vector<Point> boundary, std::vector<Point> sorted_vertices, std::vector<Polygon> obstacles){
	float lower_limit = -1;
	float upper_limit = -1;

	for (const Point& pt : boundary){
		if (lower_limit == -1){
			lower_limit = pt.y;
		} else {
			if (pt.y < lower_limit){
				lower_limit = pt.y;
			}
		}

		if (upper_limit == -1){
			upper_limit = pt.y;
		} else {
			if (pt.y > upper_limit){
				upper_limit = pt.y;
			}
		}
	}

	Point temp_point;
	std::vector<Point> curr_segment;
	std::vector<Point> temp_segment;
	std::vector< std::vector<Point> > created_segments;

	for (int vertex_index = 0; vertex_index < sorted_vertices.size(); vertex_index++){
		bool up = false;
		bool down = false;
		bool will_brake = false;
		Point lower_obs_pt;
		Point upper_obs_pt;
		Point intersection_point;

		curr_segment = {};
		temp_point.x = sorted_vertices[vertex_index].x;
		temp_point.y = lower_limit;
		curr_segment.emplace_back(temp_point);
		lower_obs_pt = temp_point;

		temp_point.y = upper_limit;
		curr_segment.emplace_back(temp_point);
		upper_obs_pt = temp_point;

		int counter_vertex = 0;
		int previous_vertex = 0;
		// iterate all the vertices of the obstacles and check if they intersect with other obstacle lines
		for (int obs = 0; obs < obstacles.size()&&!will_brake; obs++){
			previous_vertex = counter_vertex;
			for (int vertex = 0; vertex < obstacles[obs].size()-1&&!will_brake; vertex++){
				counter_vertex++;
				if (vertex != obstacles.size() - 1){
					temp_segment = {obstacles[obs][vertex], obstacles[obs][vertex + 1]};
				} else {
					temp_segment = {obstacles[obs][vertex], obstacles[obs][0]};
				}
				if (intersection_segment_segment(curr_segment[0], curr_segment[1], temp_segment[0], temp_segment[1])){
					intersection_point = get_intersection_point_segment_segment(curr_segment[0], curr_segment[1], temp_segment[0], temp_segment[1]);
					// check if the current vertex is from the current obstacle
					if (vertex_index > previous_vertex - 1 && vertex_index < previous_vertex - 1 + obstacles.size()){
						if ((intersection_point.x != sorted_vertices[vertex_index].x) || (intersection_point.y != sorted_vertices[vertex_index].y)){
							if (intersection_point.y > sorted_vertices[vertex_index].y){
								up = true;
							}
							if (intersection_point.y < sorted_vertices[vertex_index].y){
								down = true;
							}
						}
					} else {
						if ((intersection_point.x != sorted_vertices[vertex_index].x) || (intersection_point.y != sorted_vertices[vertex_index].y)){
							// make the intersection point the upper point of the vertical line
							if ((!up) && (intersection_point.y > sorted_vertices[vertex_index].y) && (intersection_point.y < upper_obs_pt.y)){
								upper_obs_pt = intersection_point;
							}
							// make the intersection point the lower point of the vertical line
							if ((!down) && (intersection_point.y < sorted_vertices[vertex_index].y) && (intersection_point.y > lower_obs_pt.y)){
								lower_obs_pt = intersection_point;
							}
						}
					}
				}
				if (up && down){
					will_brake = true;
				}
			}
		}

		temp_segment = {Point(-1,-1), Point(-1,-1)};

		if (down && !up){
			temp_segment[0] = Point(upper_obs_pt.x, upper_limit);
			temp_segment[1] = upper_obs_pt;
		} else if (up && !down){
			temp_segment[0] = lower_obs_pt;  
			temp_segment[1] = Point(upper_obs_pt.x, lower_limit);  
		} else if(!up && !down){
			temp_segment[0] = lower_obs_pt;
			temp_segment[1] = upper_obs_pt;  
		}

		if (!down || !up){
			created_segments.push_back(temp_segment);
		}
	}
	return created_segments;
}

