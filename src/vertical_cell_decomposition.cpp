#include "vertical_cell_decomposition.hpp"

std::vector< SEGMENT > create_segments_vertical_decomposition(std::vector<POINT> vertices, std::vector< std::vector<POINT> > obstacles, float limit_lower, float limit_upper){
    POINT temp_point;
    SEGMENT curr_segment;
    SEGMENT temp_segment;
    std::vector< SEGMENT > created_segments;

    for(const POINT& pt : vertices) {
        bool up = false;
        bool down = false;
        bool will_brake = false;
        POINT lower_obs_pt;
        POINT upper_obs_pt;
        POINT intersection_point;
      
        temp_point.x = pt.x;
        temp_point.y = y_limit_lower;
        temp_point.obs = pt.obs;
        curr_segment.a = temp_point;
        lower_obs_pt = temp_point;
    
        temp_point.y = y_limit_upper;
        curr_segment.b = temp_point;
        upper_obs_pt = temp_point;
	
	// iterate all the vertices of the obstacles and check if they intersect with other obstacle lines
        for(int obs = 0; obs < obstacles.size()&&!break_now; obs++) { 
          for(int vertex = 0; vertex < obstacles[obs].size()-1&&!break_now; vertex++) {
            temp_segment.a = obstacles[obs][vertex];
            temp_segment.b = obstacles[obs][vertex + 1];
            does_intersect = intersection_segment_segment(curr_segment.a, curr_segment.b, temp_segment.a, temp_segment.b);

            if(does_intersect){ 
		intersection_point = intersection_point_segment_segment, (curr_segment.a, curr_segment.b, temp_segment.a, temp_segment.b);
              // check if the current vertex is from the current obstacle
              if(obs == pt.obs) {
                // check if the intersection point is not the current point
                if((intersection_point.x != pt.x) || (intersection_point.y != pt.y)) {
                    if(intersection_point.y > pt.y) {up = true;}
                    if(intersection_point.y < pt.y) {down = true;}
                }
              }
              else {
                if((intersection_point.x != pt.x) || (intersection_point.y != pt.y)) {
                  // make the intersection point the upper point of the vertical line
                  if((!up) && (intersection_point.y > pt.y) && (intersection_point.y < upper_obs_pt.y)) {
                      upper_obs_pt = intersection_point;
                  }
                  // make the intersection point the lower point of the vertical line
                  if((!down) && (intersection_point.y < pt.y) && (intersection_point.y > lower_obs_pt.y)) {
                      lower_obs_pt = intersection_point;
                  }
                }
              }
            }
            if(up && down) {
              break_now = true;
            }
          }
        }

        temp_segment = {{-1,-1}, {-1,-1}};

        if(down && !up) {
            temp_segment.b = upper_obs_pt;
        }
        else if(up && !down) {
            temp_segment.a = lower_obs_pt;  
        }
        else if(!up && !down) {
            temp_segment.a = lower_obs_pt;
            temp_segment.b = upper_obs_pt;  
        } 
        open_line_segments.push_back(temp_segment);
    }
    return open_line_segments;
}

