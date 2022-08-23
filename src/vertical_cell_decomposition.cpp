#include "vertical_cell_decomposition.hpp"


std::vector< std::tuple<Point, int> > sort_vertices(std::vector<Polygon> obstacles){
	float vertices_num;
	std::vector <std::tuple<Point, int> > sorted_vertices;
	bool add_to_list;
	for (int i = 0; i < obstacles.size(); i++){
		for (const auto &point : obstacles[i]){
			vertices_num += 1;
			sorted_vertices.push_back(std::make_tuple(Point(-1, -1), -1));
		}
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

					for (int vert = 0; vert < sorted_vertices.size(); vert++){
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
		for (int obs = 0; obs < obstacles.size() && !will_brake; obs++){
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


float cell_area(Polygon cell){
	float area = 0;
	int vertices_num = 4;
	for (int i = 0; i < vertices_num; i += 2){
		if (i+2 < vertices_num){
			area += cell[i+1].x * (cell[i+2].y - cell[i].y) + cell[i+1].y * (cell[i].x - cell[i+2].x);
		} else {
			if (i+1 < vertices_num){
				area += cell[i+1].x * (cell[0].y - cell[i].y) + cell[i+1].y * (cell[i].x - cell[0].x);
			} else {
				area += cell[0].x * (cell[0].y - cell[i].y) + cell[0].y * (cell[i].x - cell[0].x);
			}
		}
	}
	area = area / 2;
	return area;
}


std::vector<Polygon> find_cells(std::vector<Point> boundary, std::vector <std::tuple<Point, int> > sorted_vertices, std::vector<Polygon> obstacles){
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

	std::vector<Polygon> cells;
	Point temp_point = Point(-1, -1);
	Point curr_vertex;
	Point next_vertex;
	std::vector<Point> temp_segment = {Point(-1, -1), Point(-1, -1)};
	std::vector<Point> curr_segment;
	std::vector<Point> next_segment;
	std::vector< std::vector<Point> > lines_to_check;
	std::vector<int> segment_typology;
	std::vector<Polygon> trapezoids;
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

			if (!done[2]){
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

	// merge overlapping cells
	std::vector<int> quads_to_remove;
	std::vector<Polygon> quads_to_add;
	Polygon temp1;
	Polygon temp2;
	Polygon new_quad;
	float area1, area2, area3;

	for (int cell1 = 0; cell1 < cells.size(); cell1++){
		for (int cell2 = 0; cell2 < cells.size(); cell2++){
			if(cell1 != cell2) {
				if (cells[cell1][0].x == cells[cell2][0].x && cells[cell1][1].x == cells[cell2][1].x){
					temp1 = cells[cell1];
					temp2 = cells[cell2];
					area1 = cell_area(temp1);
					area2 = cell_area(temp2);

					temp_point.x = temp1[0].x;
					temp_point.y = std::min(temp1[0].y, temp2[0].y);
					new_quad.push_back(temp_point);
					temp_point.x = temp1[1].x;
					temp_point.y = std::min(temp1[1].y, temp2[1].y);
					new_quad.push_back(temp_point);
					temp_point.x = temp1[1].x;
					temp_point.y = std::max(temp1[2].y, temp2[2].y);
					new_quad.push_back(temp_point);
					temp_point.x = temp1[0].x;
					temp_point.y = std::max(temp1[3].y, temp2[3].y);
					new_quad.push_back(temp_point);
					area3 = cell_area(new_quad);

					if (area1 + area2 >= area3) {
						quads_to_remove.push_back(cell1);
						quads_to_remove.push_back(cell2);
						quads_to_add.push_back(new_quad);
					}
					new_quad = {};
				}
			}
		}
	}
	std::sort(quads_to_remove.begin(), quads_to_remove.end());
	quads_to_remove.erase(std::unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
	for (int quad = quads_to_remove.size() - 1; quad >= 0; quad--){
		cells.erase(cells.begin() + quads_to_remove[quad]);
	}
	for (int quad = 0; quad < quads_to_add.size(); quad++){
		cells.push_back(quads_to_add[quad]);
	}

	quads_to_remove = {};
	for (int quad1 = 0; quad1 < cells.size(); quad1++){
		for (int quad2 = quad1 + 1; quad2 < cells.size(); quad2++){
			bool duplicate = true;
			for (int point = 0; point < cells[quad1].size(); point++){
				if ((cells[quad1][point].x != cells[quad2][point].x) || (cells[quad1][point].y != cells[quad2][point].y)){
					duplicate = false;
					break;
				}
			}
			if (duplicate){
				quads_to_remove.push_back(quad2);
			}
		}
	}
	std::sort(quads_to_remove.begin(), quads_to_remove.end());
	quads_to_remove.erase(std::unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
	for (int quad = quads_to_remove.size() - 1; quad >= 0; quad--){
		cells.erase(cells.begin() + quads_to_remove[quad]);
	}

	quads_to_remove = {};
	for (int quad1 = 0; quad1 < cells.size(); quad1 ++){
		for (int quad2 = 0; quad2 < cells.size(); quad2 ++){
			if (quad1 != quad2 && cells[quad1][0].x == cells[quad2][0].x && cells[quad1][1].x == cells[quad2][1].x){
				if ((cells[quad1][0].y <= cells[quad2][0].y) && (cells[quad1][1].y <= cells[quad2][1].y) && (cells[quad1][2].y >= cells[quad2][2].y) && (cells[quad1][3].y >= cells[quad2][3].y)){
					quads_to_remove.push_back(quad2);
				}
			}
		}
	}
	std::sort(quads_to_remove.begin(), quads_to_remove.end());
	quads_to_remove.erase(std::unique(quads_to_remove.begin(), quads_to_remove.end()), quads_to_remove.end());
	for (int quad = quads_to_remove.size() - 1; quad >= 0; quad--){
		cells.erase(cells.begin() + quads_to_remove[quad]);
	}

	// Add boundary cells
	Polygon new_cell = {};
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


std::tuple< std::vector<Point>, std::vector< std::vector<float> > > create_roadmap(std::vector<Polygon> cells, std::vector<Polygon> obstacles, const std::vector<Polygon>& gate_list, const std::vector<float> x, const std::vector<float> y){
	std::vector<int> same_boundary;
	std::vector<Point> graph_vertices;
	std::vector< std::vector<int> > graph_edges;
	std::vector< std::vector<float> > adjacency_matrix;
	Point centroid_vertex;
	Point curr_centroid_vertex;
	std::vector<int> temp_edge;
	std::vector<Point> temp_points;
	bool inside;
	int place; 
	int place1;
	int place2;
	int use; 
	int n;

	// for each quad cell find the neigbour cells (that have a common boundary)
	for (int cell1 = 0; cell1 < cells.size(); cell1++){
		same_boundary = {};
		//compare to the rest of the cells if it is not the same cell
		for (int cell2 = 0; cell2 < cells.size(); cell2++){
			if (cell1 != cell2){
				if ((cells[cell1][1].x == cells[cell2][0].x) && 
					((cells[cell1][2].y == cells[cell2][0].y || cells[cell1][2].y == cells[cell2][3].y) ||
					(cells[cell1][1].y == cells[cell2][0].y || cells[cell1][1].y == cells[cell2][3].y))){
					same_boundary.push_back(cell2);
				}
			}
		}

		temp_points = {};
		for (int pt = 0; pt < 4; pt++){
			temp_points.push_back(cells[cell1][pt]);
		}
		centroid_vertex = get_cell_centroid(temp_points);
		inside = false;
		for (int vertex = 0; vertex < graph_vertices.size(); vertex++){
			if (centroid_vertex.x == graph_vertices[vertex].x && centroid_vertex.y == graph_vertices[vertex].y){ 
				inside = true;
				place = vertex;
				break;
			}
		}
		if (!inside){
			graph_vertices.push_back(centroid_vertex); 
			place = -1;
		}

		if (same_boundary.size() == 1){
			temp_points = {};
			temp_points.push_back(cells[cell1][1]);
			temp_points.push_back(cells[cell1][2]);
			graph_vertices.push_back(get_cell_centroid(temp_points));
			n = graph_vertices.size() - 1;

			if (place != -1){
				temp_edge = {place, n};
				// if (!get_intersection_segment_obstacles(graph_vertices[temp_edge[0]], graph_vertices[temp_edge[1]], obstacles)){
					graph_edges.push_back(temp_edge);
				// }
			} else {
				temp_edge = {n-1, n};
				// if (!get_intersection_segment_obstacles(graph_vertices[temp_edge[0]], graph_vertices[temp_edge[1]], obstacles)){
					graph_edges.push_back(temp_edge);
				// }
			}

			temp_points = {};
			for (int pt = 0; pt < 4; pt++){
				temp_points.push_back(cells[same_boundary[0]][pt]);
			}
			curr_centroid_vertex = get_cell_centroid(temp_points);
			inside = false;
			for (int vertex = 0; vertex < graph_vertices.size(); vertex++){
				if (curr_centroid_vertex.x == graph_vertices[vertex].x && curr_centroid_vertex.y == graph_vertices[vertex].y){ 
					inside = true;
					place2 = vertex;
					break;
				}
			}
			if (!inside){ 
				place2 = -1;
			}
			if (place2 == -1){
				graph_vertices.push_back(curr_centroid_vertex);
				temp_edge = {n, n+1};
				// if (!get_intersection_segment_obstacles(graph_vertices[temp_edge[0]], graph_vertices[temp_edge[1]], obstacles)){
					graph_edges.push_back(temp_edge);
				// }
			} else {
				temp_edge = {n, place2};
				// if (!get_intersection_segment_obstacles(graph_vertices[temp_edge[0]], graph_vertices[temp_edge[1]], obstacles)){
					graph_edges.push_back(temp_edge);
				// }
			}
		} else if (same_boundary.size() > 1){
			n = graph_vertices.size() - 1;
			if (place != -1){
				use = place;
			} else {
				use = n;
			}

			for (int i = 0; i < same_boundary.size(); i++){
				temp_points.clear();
				for(int pt = 0; pt < 4; pt++){
					temp_points.push_back(cells[same_boundary[i]][pt]);
				}
				curr_centroid_vertex = get_cell_centroid(temp_points);
				temp_points = {};
				temp_points.push_back(cells[same_boundary[i]][0]);
				temp_points.push_back(cells[same_boundary[i]][3]);
				graph_vertices.push_back(get_cell_centroid(temp_points));
				place1 = graph_vertices.size() - 1;
				inside = false;
				for (int vertex = 0; vertex < graph_vertices.size(); vertex++){
					if (curr_centroid_vertex.x == graph_vertices[vertex].x && curr_centroid_vertex.y == graph_vertices[vertex].y){ 
						inside = true;
						place2 = vertex;
					}
				}
				if (!inside){
					graph_vertices.push_back(curr_centroid_vertex);
					place2 = graph_vertices.size() - 1;
				}
				temp_edge = {use, place1};
				// if (!get_intersection_segment_obstacles(graph_vertices[temp_edge[0]], graph_vertices[temp_edge[1]], obstacles)){
					graph_edges.push_back(temp_edge);
				// }
				temp_edge = {place1, place2};
				// if (!get_intersection_segment_obstacles(graph_vertices[temp_edge[0]], graph_vertices[temp_edge[1]], obstacles)){
					graph_edges.push_back(temp_edge);
				// }
			}
		}
	}

	// Add starting robot positions
	for (int i = 0; i < x.size(); i++){
		graph_vertices.push_back(Point(x[i], y[i]));
	}

	// Add ending gates
	for (int i = 0; i < gate_list.size(); i++){
		graph_vertices.push_back(get_cell_centroid(gate_list[i]));
	}

	// construct the adjacency matrix
	adjacency_matrix = {};
	for (int i = 0; i < graph_vertices.size(); i++){
		adjacency_matrix.push_back({});
		for (int j = 0; j < graph_vertices.size(); j++){
			adjacency_matrix[i].push_back(0.0);
		}
	}
	for (const std::vector<int> edge : graph_edges){
		float distance = sqrt(pow(graph_vertices[edge[0]].x - graph_vertices[edge[1]].x, 2) + pow(graph_vertices[edge[0]].y - graph_vertices[edge[1]].y, 2));
		adjacency_matrix[edge[0]][edge[1]] = distance;
		adjacency_matrix[edge[1]][edge[0]] = distance;
	}

	// add all the possible edges in order to connect starting points, ending points and obtaining better routes
	for (int i = 0; i < adjacency_matrix.size(); i++){
		for (int j = 0; j < adjacency_matrix[i].size(); j++){
			if (!get_intersection_segment_obstacles(graph_vertices[i], graph_vertices[j], obstacles)){
				float distance = sqrt(pow(graph_vertices[i].x - graph_vertices[j].x, 2) + pow(graph_vertices[i].y - graph_vertices[j].y, 2));
				adjacency_matrix[i][j] = distance;
				adjacency_matrix[j][i] = distance;
			}
		}
	}
	return std::make_tuple(graph_vertices, adjacency_matrix);
}

