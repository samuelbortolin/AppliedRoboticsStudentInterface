#include "vertical_cell_decomposition.hpp"


std::vector <std::tuple<Point, int> > sort_vertices(std::vector< std::vector<Point> > obstacles){
	float vertices_num;
	for (int i = 0; i < obstacles.size(); i++){
		for (const auto &point : obstacles[i]){
			vertices_num += 1;
		}
	}

	std::vector <std::tuple<Point, int> > sorted_vertices;
	bool add_to_list;

	for (int curr_vertex = 0; curr_vertex < vertices_num; curr_vertex++){
		sorted_vertices.push_back(std::make_tuple(Point(-1, -1), -1));
	}

	for (int curr_vertex = vertices_num - 1; curr_vertex >= 0; curr_vertex--){
		for (int obj = 0; obj < obstacles.size(); obj++){
			for (int vertex = 0; vertex < obstacles[obj].size(); vertex++){
				add_to_list = false;
				if (obstacles[obj][vertex].x > std::get<0>(sorted_vertices[curr_vertex]).x){
					if (curr_vertex == vertices_num - 1 ||
						obstacles[obj][vertex].x < std::get<0>(sorted_vertices[curr_vertex + 1]).x ||
						obstacles[obj][vertex].x == std::get<0>(sorted_vertices[curr_vertex + 1]).x){
						add_to_list = true;
					}

					for (int vert = 0; vert < sorted_vertices.size(); vert ++){
						if (std::get<0>(sorted_vertices[vert]).x == obstacles[obj][vertex].x && std::get<0>(sorted_vertices[vert]).y == obstacles[obj][vertex].y){
							add_to_list = false;
						}
					}
				}
				if (add_to_list){
					sorted_vertices[curr_vertex] = std::make_tuple(obstacles[obj][vertex], obj);
				}
			}
		}
	}
	return sorted_vertices;
}


std::vector< std::vector<Point> > create_segments_vertical_decomposition(std::vector <std::tuple<Point, int> > sorted_vertices, std::vector<Polygon> obstacles, float lower_limit, float upper_limit){
	Point temp_point;
	std::vector<Point> curr_segment;
	std::vector<Point> temp_segment;
	std::vector< std::vector<Point> > created_segments;

	for (int vertex_index = 0; vertex_index < sorted_vertices.size(); vertex_index++){
		bool up = false;
		bool low = false;
		bool will_brake = false;
		Point lower_obs_pt;
		Point upper_obs_pt;
		Point intersection_point;

		curr_segment = {};
		temp_point.x = std::get<0>(sorted_vertices[vertex_index]).x;
		temp_point.y = lower_limit;
		curr_segment.emplace_back(temp_point);
		lower_obs_pt = temp_point;
		temp_point.y = upper_limit;
		curr_segment.emplace_back(temp_point);
		upper_obs_pt = temp_point;

		// iterate all the vertices of the obstacles and check if they intersect with other obstacle lines
		for (int obs = 0; obs < obstacles.size()&&!will_brake; obs++){
			for (int vertex = 0; vertex < obstacles[obs].size() && !will_brake; vertex++){
				if (vertex != obstacles[obs].size() - 1){
					temp_segment = {obstacles[obs][vertex], obstacles[obs][vertex + 1]};
				} else {
					temp_segment = {obstacles[obs][vertex], obstacles[obs][0]};
				}
				if (intersection_segment_segment(curr_segment[0], curr_segment[1], temp_segment[0], temp_segment[1])){
					intersection_point = get_intersection_point_segment_segment(curr_segment[0], curr_segment[1], temp_segment[0], temp_segment[1]);
					// check if the current vertex is from the current obstacle
					if (obs == std::get<1>(sorted_vertices[vertex_index])){
						if ((intersection_point.x != std::get<0>(sorted_vertices[vertex_index]).x) || (intersection_point.y != std::get<0>(sorted_vertices[vertex_index]).y)){
							if (intersection_point.y > std::get<0>(sorted_vertices[vertex_index]).y){
								up = true;
							} else if (intersection_point.y < std::get<0>(sorted_vertices[vertex_index]).y){
								low = true;
							}
						}
					} else {
						if ((intersection_point.x != std::get<0>(sorted_vertices[vertex_index]).x) || (intersection_point.y != std::get<0>(sorted_vertices[vertex_index]).y)){
							// the intersection point will be the upper or the lower point of the vertical line
							if ((!up) && (intersection_point.y > std::get<0>(sorted_vertices[vertex_index]).y) && (intersection_point.y < upper_obs_pt.y)){
								upper_obs_pt = intersection_point;
							} else if ((!low) && (intersection_point.y < std::get<0>(sorted_vertices[vertex_index]).y) && (intersection_point.y > lower_obs_pt.y)){
								lower_obs_pt = intersection_point;
							}
						}
					}
				}
				if (up && low){
					will_brake = true;
				}
			}
		}

		temp_segment = {Point(-1,-1), Point(-1,-1)};

		if (low && !up){
			temp_segment[1] = upper_obs_pt;
		} else if (up && !low){
			temp_segment[0] = lower_obs_pt;
		} else if(!up && !low){
			temp_segment[0] = lower_obs_pt;
			temp_segment[1] = upper_obs_pt;
		}

		created_segments.push_back(temp_segment);
	}
	return created_segments;
}


std::vector<Polygon> find_cells(std::vector<Point> boundary, std::vector <std::tuple<Point, int> > sorted_vertices, std::vector< std::vector<Point> > obstacles){
	float lower_limit = -1;
	float upper_limit = -1;
	float left_limit = -1;
	float right_limit = -1;

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

		if (left_limit == -1){
			left_limit = pt.x;
		} else {
			if (pt.y < left_limit){
				left_limit = pt.x;
			}
		}

		if (right_limit == -1){
			right_limit = pt.x;
		} else {
			if (pt.y > right_limit){
				right_limit = pt.x;
			}
		}
	}

	std::vector< std::vector<Point> > segments = create_segments_vertical_decomposition(sorted_vertices, obstacles, lower_limit, upper_limit);

	std::vector< std::vector<Point> > cells;
	Point temp_point = Point(-1, -1);
	Point curr_vertex;
	Point next_vertex;
	std::vector<Point> temp_segment = {Point(-1, -1), Point(-1, -1)};
	std::vector<Point> curr_segment;
	std::vector<Point> next_segment;
	std::vector< std::vector<Point> > lines_to_check;
	std::vector<int> segment_typology;
	std::vector< std::vector<Point> > trapezoids;
	std::vector<bool> done;

	for (int i = 0; i < segments.size(); i++){
		curr_segment = segments[i];
		curr_vertex = std::get<0>(sorted_vertices[i]);
		done = {false, false, true};

		if (curr_segment[0].y == -1){
			done[0] = true;
		}

		if (curr_segment[1].y == -1){
			done[1] = true;
		}

		if ((curr_segment[0].y == -1) && (curr_segment[1].y == -1)){
			done[2] = false;
		}

		for (int j = i+1; j < segments.size(); j++){
			lines_to_check = {};
			segment_typology = {};
			trapezoids = {};
			next_segment = segments[j];
			next_vertex = std::get<0>(sorted_vertices[j]);

			if (!done[0]){
				if (next_segment[0].y != -1 && next_segment[1].y != -1){
					temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex});
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(0);
					trapezoids.push_back({curr_segment[0], next_segment[0], next_vertex, curr_vertex});
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(0);
					trapezoids.push_back({curr_segment[0], next_vertex, next_segment[1], curr_vertex});
				} else if (next_segment[0].y != -1){
					temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex});
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(0);
					trapezoids.push_back({curr_segment[0], next_segment[0], next_vertex, curr_vertex});
				} else if (next_segment[1].y != -1){
					temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex});
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(0);
					trapezoids.push_back({curr_segment[0], next_vertex, next_segment[1], curr_vertex});
				} else {
					temp_segment[0] = get_cell_centroid({curr_segment[0], curr_vertex});
					temp_segment[1] = next_vertex;
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(0);
					trapezoids.push_back({curr_segment[0], next_vertex, curr_vertex});
				}
			}

			if (!done[1]){
				if (next_segment[0].y != -1 && next_segment[1].y != -1){
					temp_segment[0] = get_cell_centroid({curr_segment[1], curr_vertex});
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(1);
					trapezoids.push_back({curr_vertex, next_segment[0], next_vertex, curr_segment[1]});
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(1);
					trapezoids.push_back({curr_vertex, next_vertex, next_segment[1], curr_segment[1]});
				} else if (next_segment[0].y != -1){
					temp_segment[0] = get_cell_centroid({curr_segment[1], curr_vertex});
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(1);
					trapezoids.push_back({curr_vertex,next_segment[0],next_vertex,curr_segment[1]});
				} else if (next_segment[1].y != -1){
					temp_segment[0] = get_cell_centroid({curr_segment[1], curr_vertex});
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(1);
					trapezoids.push_back({curr_vertex, next_vertex, next_segment[1], curr_segment[1]});
				} else {
					temp_segment[0] = get_cell_centroid({curr_segment[1], curr_vertex});
					temp_segment[1] = next_vertex;
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(1);
					trapezoids.push_back({curr_vertex, next_vertex, curr_segment[1]});
				}
			}

			if (done[2] == 0){
				if (next_segment[0].y != -1 && next_segment[1].y != -1){
					temp_segment[0] = curr_vertex;
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(2);
					trapezoids.push_back({curr_vertex, next_segment[0], next_vertex});
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(2);
					trapezoids.push_back({curr_vertex, next_vertex, next_segment[1]});
				} else if (next_segment[0].y != -1){
					temp_segment[0] = curr_vertex;
					temp_segment[1] = get_cell_centroid({next_segment[0], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(2);
					trapezoids.push_back({curr_vertex, next_segment[0], next_vertex});
				} else if (next_segment[1].y != -1){
					temp_segment[0] = curr_vertex;
					temp_segment[1] = get_cell_centroid({next_segment[1], next_vertex});
					lines_to_check.push_back(temp_segment);
					segment_typology.push_back(2);
					trapezoids.push_back({curr_vertex, next_vertex, next_segment[1]});
				} else {
					temp_segment[0] = curr_vertex;
					temp_segment[1] = next_vertex;
					lines_to_check.push_back({temp_segment});
					segment_typology.push_back(2);
					trapezoids.push_back({curr_vertex, next_vertex});
				}
			}

			std::vector<int> temp_to_remove;
			for (int line = 0; line < lines_to_check.size(); line++){
				std::vector<bool> no_intersection = {true, true, true};
				for (int obs = 0; obs < obstacles.size(); obs++){
					for (int vertex = 0; vertex < obstacles[obs].size(); vertex++){
						temp_segment[0] = obstacles[obs][vertex];
						if (vertex != obstacles[obs].size() - 1){
							temp_segment[1] = obstacles[obs][vertex+1];
						} else {
							temp_segment[1] = obstacles[obs][0];
						}
						temp_point = get_intersection_point_segment_segment(lines_to_check[line][0], lines_to_check[line][1], temp_segment[0], temp_segment[1]);

						if (temp_point.y != -1){
							no_intersection[segment_typology[line]] = false;
							bool found = false;
							for (int idx = 0; idx < temp_to_remove.size(); idx++){
								if (line == temp_to_remove[idx]){
									found = true;
									break;
								}
							}
							if (!found){
								temp_to_remove.push_back(line);
							}
						}
					}
				}
				if (no_intersection[segment_typology[line]]){
					done[segment_typology[line]] = true;
				}
			}

			for (int line = 0; line < lines_to_check.size(); line++){
				bool found = false;
				for (int idx = 0; idx < temp_to_remove.size(); idx++){
					if (line == temp_to_remove[idx]){
						found = true;
						break;
					}
				}
				if (!found){
					cells.push_back(trapezoids[line]);
				}
			}
			if (done[0] && done[1] && done[2]){
				break;
			}
		}
	}

	std::vector<Point> new_cell = {};
	if(left_limit != std::get<0>(sorted_vertices[0]).x) {
		new_cell.push_back(Point(left_limit, lower_limit));

		temp_point.x = std::get<0>(sorted_vertices[0]).x;
		temp_point.y = lower_limit;
		new_cell.push_back(temp_point);

		temp_point.x = std::get<0>(sorted_vertices[0]).x;
		temp_point.y = upper_limit;
		new_cell.push_back(temp_point);

		new_cell.push_back(Point(left_limit, upper_limit));

		cells.push_back(new_cell);
	}

	new_cell = {};
	if(right_limit != std::get<0>(sorted_vertices[sorted_vertices.size() - 1]).x) {
		temp_point.x = std::get<0>(sorted_vertices[sorted_vertices.size() - 1]).x;
		temp_point.y = lower_limit;
		new_cell.push_back(temp_point);

		new_cell.push_back(Point(right_limit, lower_limit));

		new_cell.push_back(Point(right_limit, upper_limit));

		temp_point.x = std::get<0>(sorted_vertices[sorted_vertices.size() - 1]).x;
		temp_point.y = upper_limit;
		new_cell.push_back(temp_point);

		cells.push_back(new_cell);
	}

	return cells;
}


Point get_cell_centroid(Polygon cell){
	int cell_vertices = cell.size();

	if (cell_vertices == 0){
		return Point(-1, -1);
	}

	float sum_over_x = 0;
	float sum_over_y = 0;
	for (int vertex_index = 0; vertex_index < cell_vertices; vertex_index++){
		sum_over_x += cell[vertex_index].x;
		sum_over_y += cell[vertex_index].y;
	}

	return Point(sum_over_x/float(cell_vertices), sum_over_y/float(cell_vertices));
}


// TODO: add the method to merge overlapping cells
// std::vector<Polygon> merge_cells(std::vector<Polygon> cells);

// TODO: add the method to create the roadmap
// std::tuple< std::vector<Point>, std::vector< std::vector<int> > > create_roadmap(std::vector< std::vector<Point> > cells, std::vector< std::vector<Point> > obstacles);

