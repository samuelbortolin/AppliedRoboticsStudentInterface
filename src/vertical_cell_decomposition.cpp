#include "vertical_cell_decomposition.hpp"


std::vector< std::vector<Point> > create_segments_vertical_decomposition(std::vector<Point> boundary, std::vector<Polygon> obstacles){
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
			if (pt.y < upper_limit){
				upper_limit = pt.y;
			}
		}
	}


	Point temp_point;
	std::vector<Point> curr_segment;
	std::vector<Point> temp_segment;
	std::vector< std::vector<Point> > created_segments;

	for (int obs_1 = 0; obs_1 < obstacles.size(); obs_1++){
		for (const Point& pt : obstacles[obs_1]){
			bool up = false;
			bool down = false;
			bool will_brake = false;
			Point lower_obs_pt;
			Point upper_obs_pt;
			Point intersection_point;

			curr_segment = {};
			temp_point.x = pt.x;
			temp_point.y = lower_limit;
			curr_segment.emplace_back(temp_point);
			lower_obs_pt = temp_point;

			temp_point.y = upper_limit;
			curr_segment.emplace_back(temp_point);
			upper_obs_pt = temp_point;

			// iterate all the vertices of the obstacles and check if they intersect with other obstacle lines
			for (int obs = 0; obs < obstacles.size()&&!will_brake; obs++){ 
				for (int vertex = 0; vertex < obstacles[obs].size()-1&&!will_brake; vertex++){
					temp_segment = {obstacles[obs][vertex], obstacles[obs][vertex + 1]};
					if (intersection_segment_segment(curr_segment[0], curr_segment[1], temp_segment[0], temp_segment[1])){
						intersection_point = get_intersection_point_segment_segment(curr_segment[0], curr_segment[1], temp_segment[0], temp_segment[1]);
						// check if the current vertex is from the current obstacle
						if (obs == obs_1){
							// check if the intersection point is not the current point
							if ((intersection_point.x != pt.x) || (intersection_point.y != pt.y)){
								if(intersection_point.y > pt.y) {up = true;}
								if(intersection_point.y < pt.y) {down = true;}
							}
						} else {
							if ((intersection_point.x != pt.x) || (intersection_point.y != pt.y)){
								// make the intersection point the upper point of the vertical line
								if ((!up) && (intersection_point.y > pt.y) && (intersection_point.y < upper_obs_pt.y)){
									upper_obs_pt = intersection_point;
								}
								// make the intersection point the lower point of the vertical line
								if ((!down) && (intersection_point.y < pt.y) && (intersection_point.y > lower_obs_pt.y)){
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
				temp_segment[1] = upper_obs_pt;
			} else if (up && !down){
				temp_segment[0] = lower_obs_pt;  
			} else if(!up && !down){
				temp_segment[0] = lower_obs_pt;
				temp_segment[1] = upper_obs_pt;  
			} 
			created_segments.push_back(temp_segment);
		}
	}
	return created_segments;
}

