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


std::vector< std::vector<Point> > find_cells(std::vector<Point> boundary, std::vector< std::vector<Point> > segments, std::vector<Point> sorted_vertices, std::vector< std::vector<Point> > obstacles){
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
	std::vector<Point> temp_segment = {Point(-1, -1), Point(-1, -1)};
	Point curr_vertex;
	std::vector<Point> curr_segment;
	std::vector<Point> next_segment;
	std::vector<Point> next_next_segment;
	Point next_vertex;
	Point next_next_vertex;
	std::vector< std::vector<Point> > lines_to_check;
	std::vector<int> group;
	std::vector< std::vector<Point> > trapezoids;
	std::vector<Point> temp_points;
	std::vector< std::vector<Point> > cells;
	std::vector<int> done;
	bool extra_search = true;

	for (int i = 0; i < segments.size(); i++){
		curr_segment = segments[i];
		curr_vertex = sorted_vertices[i];
		done = {false, false};
		if (curr_segment[0].y == upper_limit){
			done[0] = true;
		}
		if (curr_segment[1].y == lower_limit){
			done[1] = true;
		}

		int counter = 0;
		for (int j = i+1; j < segments.size(); j++){
			counter++;
			lines_to_check.clear();
			group.clear();
			trapezoids.clear();
			bool double_check = false;
			bool next_two_seg_same_x = false;

			next_segment = segments[j];
			next_vertex = sorted_vertices[j];
			if (j != segments.size()-1 && extra_search){
				next_next_segment = segments[j+1];
				next_next_vertex = sorted_vertices[j+1];
				next_two_seg_same_x = (next_vertex.x == next_next_vertex.x);
			}
			double_check = next_segment[0].y != upper_limit && next_segment[1].y != lower_limit;

			if (!done[0]){
				if (double_check){
					temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex}); 
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex}); 
					lines_to_check.push_back(temp_segment);
					group.push_back(0);            
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex}); 
					lines_to_check.push_back(temp_segment);
					group.push_back(0);
					trapezoids.push_back({curr_segment[0], next_segment[0], next_vertex, curr_vertex});
					trapezoids.push_back({curr_segment[0], next_vertex, next_segment[1], curr_vertex});
				} else if (next_segment[0].y != upper_limit){
					temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex}); 
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex}); 
					lines_to_check.push_back(temp_segment);
					group.push_back(0);
					trapezoids.push_back({curr_segment[0], next_segment[0], next_vertex, curr_vertex});
					if (extra_search){
						if (next_two_seg_same_x && (next_next_segment[1].y != lower_limit) && (next_next_segment[0].y == upper_limit)){
							temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex}); 
							temp_segment[1] = get_cell_centroid({next_next_segment[1], next_next_vertex}); 
							lines_to_check.push_back(temp_segment);
							group.push_back(0);
							trapezoids.push_back({curr_segment[0], next_next_vertex, next_next_segment[1], curr_vertex});              
						}
					}
				} else if (next_segment[1].y != lower_limit){
					temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex}); 
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex}); 
					lines_to_check.push_back(temp_segment);
					group.push_back(0);
					trapezoids.push_back({curr_segment[0], next_vertex, next_segment[1], curr_vertex});
					if(extra_search){
						if(next_two_seg_same_x && (next_next_segment[0].x != upper_limit) && (next_next_segment[1].x == lower_limit)){
							temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex}); 
							temp_segment[1] = get_cell_centroid({next_next_segment[0], next_next_vertex}); 
							lines_to_check.push_back(temp_segment);
							group.push_back(0);
							trapezoids.push_back({curr_segment[0], next_next_segment[0], next_next_vertex, curr_vertex});              
						}
					}
				} else {
					temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex}); 
					temp_segment[1] = next_vertex;
					lines_to_check.push_back(temp_segment);
					group.push_back(0);
					trapezoids.push_back({curr_segment[0], next_vertex, curr_vertex});
				}
			}

			if (!done[1]){
				if(double_check) {
					temp_segment[0] = get_cell_centroid({curr_segment[1], curr_vertex}); 
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex}); 
					lines_to_check.push_back(temp_segment);
					group.push_back(1);         
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex}); 
					lines_to_check.push_back(temp_segment);
					group.push_back(1);
					trapezoids.push_back({curr_vertex, next_segment[0], next_vertex, curr_segment[1]});
					trapezoids.push_back({curr_vertex, next_vertex, next_segment[1], curr_segment[1]});
				}
				else if(next_segment[0].y != upper_limit) {
					temp_segment[0] = get_cell_centroid({curr_segment[1], curr_vertex}); 
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex}); 
					lines_to_check.push_back(temp_segment);
					group.push_back(1);
					trapezoids.push_back({curr_vertex,next_segment[0],next_vertex,curr_segment[1]});
					if(extra_search){
						if(next_two_seg_same_x && (next_next_segment[1].x != lower_limit)  && (next_next_segment[0].x == upper_limit)){
							temp_segment[0] = get_cell_centroid({curr_segment[1],curr_vertex}); 
							temp_segment[1] = get_cell_centroid({next_next_segment[1],next_next_vertex});
							lines_to_check.push_back(temp_segment);
							group.push_back(1);
							trapezoids.push_back({curr_vertex, next_next_vertex, next_next_segment[1], curr_segment[1]});              
						}
					}
				}
				else if(next_segment[1].y != lower_limit) {
					temp_segment[0] = get_cell_centroid({curr_segment[1], curr_vertex}); 
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex}); 
					lines_to_check.push_back(temp_segment);
					group.push_back(1);
					trapezoids.push_back({curr_vertex, next_vertex, next_segment[1], curr_segment[1]});
					if(extra_search){
						if(next_two_seg_same_x && (next_next_segment[0].x != upper_limit)  && (next_next_segment[1].x == lower_limit)){
							temp_segment[0] = get_cell_centroid({curr_segment[1], curr_vertex}); 
							temp_segment[1] = get_cell_centroid({next_next_segment[0], next_next_vertex});
							lines_to_check.push_back(temp_segment);
							group.push_back(1);
							trapezoids.push_back({curr_vertex, next_next_segment[0], next_next_vertex,curr_segment[1]});              
						}
					}
				}
				else {
					temp_segment[0] = get_cell_centroid({curr_segment[1], curr_vertex}); 
					temp_segment[1] = next_vertex;
					lines_to_check.push_back(temp_segment);
					group.push_back(1);
					trapezoids.push_back({curr_vertex, next_vertex, curr_segment[1]});
				}
			}

			std::vector<int> temp_to_remove;
			for (int line = 0; line < lines_to_check.size(); line++){  
				int no_intersection[3] = {1, 1, 1};           
				for (int obs = 0; obs < obstacles.size(); obs++){              
					for (int vertex = 0; vertex < obstacles[obs].size()-1; vertex++){
						temp_segment[0] = obstacles[obs][vertex];
						temp_segment[1] = obstacles[obs][vertex+1];          
						temp_point = get_intersection_point_segment_segment(lines_to_check[line][0], lines_to_check[line][1], temp_segment[0], temp_segment[1]);

						if (temp_point.y != -1){
							no_intersection[group[line]] = 0;
							int found = 0;
							for (int idx = 0; idx < temp_to_remove.size(); idx++){
								if (line == temp_to_remove[idx]){
									found = 1;
									break;
								}
							}
							if(found == 0) {
								temp_to_remove.push_back(line);
							} 
						}
					}      
				}  
				if (no_intersection[group[line]] == 1){
					done[group[line]] = 1;
				}
			}
			for (int line = 0; line < lines_to_check.size(); line++){
				int found = 0;
				for (int idx = 0; idx < temp_to_remove.size(); idx++){
					if (line == temp_to_remove[idx]){
						found = 1;
						break;
					}
				}
				if (found == 0){
					cells.push_back(trapezoids[line]);
				}
			}
			if(done[0] && done[1] && done[2]){
				break;
			}
		}
	}
	return cells;
}


Point get_cell_centroid(std::vector<Point> cell){
	int cell_vertices = cell.size();

	if (cell_vertices == 0){
		return Point(-1, -1);
	}

	float sum_x = 0;
	float sum_y = 0;
	for (int vertex_index = 0; vertex_index < cell_vertices; vertex_index++){
		sum_x += cell[vertex_index].x;
		sum_y += cell[vertex_index].y;
	}

	return Point(sum_x/cell_vertices, sum_y/cell_vertices);
}

