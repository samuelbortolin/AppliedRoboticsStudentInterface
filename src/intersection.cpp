#include "intersection.hpp"


bool intersection_arc_segment(Point a, double r, double s, double e, Point s2, Point e2){
	double p1 = 2*s2.x*e2.x;
	double p2 = 2*s2.y*e2.y;
	double p3 = 2*a.x*s2.x;
	double p4 = 2*a.x*e2.x;
	double p5 = 2*a.y*s2.y;
	double p6 = 2*a.y*e2.y;

	double c1 = pow(s2.x,2)+pow(e2.x,2)-p1+pow(s2.y,2)+pow(e2.y,2)-p2;
	double c2 = -2*pow(e2.x,2)+p1-p3+p4-2*pow(e2.y,2)+p2-p5+p6;
	double c3 = pow(e2.x,2)-p4+pow(a.x,2)+pow(e2.y,2)-p6+pow(a.y,2)-pow(r,2);

	double delta = pow(c2,2)-4*c1*c3;
	double t1, t2, x, y;

	if (delta<0){
		return false;
	}
	else{
		if (delta>0){
			double deltaSq = sqrt(delta);
			t1 = (-c2+deltaSq)/(2*c1);
			t2 = (-c2-deltaSq)/(2*c1);
		}
		else{
			t1 = -c2/(2*c1);
			t2 = t1;
		}
	}

	if (t1>=0 && t1<=1){
		x = s2.x*t1+e2.x*(1-t1);
		y = s2.y*t1+e2.y*(1-t1);
		float theta = atan2(y-a.y, x-a.x);
		theta = theta - 2 * M_PI * floor(theta / (2 * M_PI) );
		if (s < e && theta >= s && theta <= e){
			return true;
		}
		if (s > e && !(theta > e && theta < s)){
			return true;
		}
	}

	if (t2 >=0 && t2<=1 && t2!=t1){
		x = s2.x*t2+e2.x*(1-t2);
		y = s2.y*t2+e2.y*(1-t2);
		float theta = atan2(y-a.y, x-a.x);
		theta = theta - 2 * M_PI * floor(theta / (2 * M_PI) );
		if (s < e && theta >= s && theta <= e){
			return true;
		}
		if (s > e && !(theta > e && theta < s)){
			return true;
		}

	}

	return false;
}


// Decide if angle within an arc
bool intersection_arc_pass(float theta, double s, double e){
	if (s < e && theta >= s && theta <= e)
	{
		return true;
	}
	if (s > e && !(theta > e && theta < s))
	{
		return true;
	}

	return false;
}


// Normalize an angle in range [0,2*pi)
float mod2Pi(float angle){
    return angle - 2 * M_PI * floor(angle / (2 * M_PI));
}


bool intersection_arc_arc(Point a1, double r1, double s1, double e1, Point a2, double r2, double s2, double e2){
	double eucl_distance = sqrt(pow(a1.x-a2.x,2) + pow(a1.y-a2.y,2));
	if (eucl_distance <= (r1+r2) and eucl_distance >= std::abs(r1-r2)){
		double l = (r1*r1 - r2*r2 + eucl_distance*eucl_distance) / (2*eucl_distance);
		double h = sqrt(r1*r1 - l*l);

		if (h > 0) {
			double x1 = l*(a2.x-a1.x)/eucl_distance + h*(a2.y-a1.y)/eucl_distance + a1.x;
			double y1 = l*(a2.y-a1.y)/eucl_distance - h*(a2.x-a1.x)/eucl_distance + a1.y;
			Point p1 = Point(x1, y1);
			double x2 = l*(a2.x-a1.x)/eucl_distance - h*(a2.y-a1.y)/eucl_distance + a1.x;
			double y2 = l*(a2.y-a1.y)/eucl_distance + h*(a2.x-a1.x)/eucl_distance + a1.y;
			Point p2 = Point(x2, y2);

			float theta11 = mod2Pi(atan2(p1.y-a1.y,p1.x-a1.x));
			float theta12 = mod2Pi(atan2(p1.y-a2.y,p1.x-a2.x));
			float theta21 = mod2Pi(atan2(p2.y-a1.y,p2.x-a1.x));
			float theta22 = mod2Pi(atan2(p2.y-a2.y,p1.x-a2.x));

			// both pass (x1,y1) or both pass (x2,y2)
			bool re11 = intersection_arc_pass(theta11, s1, e1);
			bool re12 = intersection_arc_pass(theta12, s2, e2);
			bool re21 = intersection_arc_pass(theta21, s1, e1);
			bool re22 = intersection_arc_pass(theta22, s2, e2);

			if ((re11 && re12) || (re21 && re22)) return true;
		} else{
			double x = l*(a2.x-a1.x)/eucl_distance + a1.x;
			double y = l*(a2.y-a1.y)/eucl_distance + a1.y;
			Point p = Point(x, y);
			float theta1 = mod2Pi(atan2(p.y-a1.y,p.x-a1.x));
			float theta2 = mod2Pi(atan2(p.y-a2.y,p.x-a2.x));

			bool re1 = intersection_arc_pass(theta1, s1, e1);
			bool re2 = intersection_arc_pass(theta2, s2, e2);

			if (re1 && re2) return true;
		}
	}

	return false;
}


bool intersection_segment_segment(Point s1, Point e1, Point s2, Point e2){
	Point intersection = get_intersection_point_segment_segment(s1, e1, s2, e2);
	if (intersection.x == -1 && intersection.y == -1){
		return false;
	} else {
		return true;
	}

}


Point get_intersection_point_segment_segment(Point s1, Point e1, Point s2, Point e2){
	float det = (e2.x-s2.x)*(s1.y-e1.y) - (s1.x-e1.x)*(e2.y-s2.y);
	if (det == 0){
		// check if the segments are on the same line
		if (e1.x - s1.x == 0){
			if (s2.x == s1.x){
				// s1 or e1 is inside s2 & e2 or vice versa
				if ((e2.y >= s1.y && s1.y >= s2.y) || 
					(s2.y >= s1.y && s1.y >= e2.y)){
					return s1;
				} else if ((e2.y >= e1.y && e1.y >= s2.y) || 
					(s2.y >= e1.y && e1.y >= e2.y)){
					return e1;
				} else if ((s1.y >= s2.y && s2.y >= e1.y) || 
					(e1.y >= s2.y && s2.y >= s1.y)){
					return s2;
				} else if ((s1.y >= e2.y && e2.y >= e1.y) || 
					(e1.y >= e2.y && e2.y >= s1.y)){
					return e2;
				} else {			
					return Point(-1, -1);
				}
			}
		} else {
			float m = (e1.y - s1.y)/(e1.x - s1.x);
			float q = e1.y - m*e1.x;
			if (s2.y == m*s2.x + q){
				// s1 or e1 is inside s2 & e2 or vice versa
				if ((e2.x >= s1.x && s1.x >= s2.x && e2.y >= s1.y && s1.y >= s2.y) || 
					(s2.x >= s1.x && s1.x >= e2.x && s2.y >= s1.y && s1.y >= e2.y) || 
					(e2.x <= s1.x && s1.x <= s2.x && e2.y >= s1.y && s1.y >= s2.y) || 
					(s2.x <= s1.x && s1.x <= e2.x && s2.y >= s1.y && s1.y >= e2.y)){
					return s1;
				} else if ((e2.x >= e1.x && e1.x >= s2.x && e2.y >= e1.y && e1.y >= s2.y) || 
					(s2.x >= e1.x && e1.x >= e2.x && s2.y >= e1.y && e1.y >= e2.y) || 
					(e2.x <= e1.x && e1.x <= s2.x && e2.y >= e1.y && e1.y >= s2.y) || 
					(s2.x <= e1.x && e1.x <= e2.x && s2.y >= e1.y && e1.y >= e2.y)){
					return e1;
				} else if ((s1.x >= s2.x && s2.x >= e1.x && s1.y >= s2.y && s2.y >= e1.y) || 
					(e1.x >= s2.x && s2.x >= s1.x && e1.y >= s2.y && s2.y >= s1.y) || 
					(s1.x <= s2.x && s2.x <= e1.x && s1.y >= s2.y && s2.y >= e1.y) || 
					(e1.x <= s2.x && s2.x <= s1.x && e1.y >= s2.y && s2.y >= s1.y)){
					return s2;
				} else if ((s1.x >= e2.x && e2.x >= e1.x && s1.y >= e2.y && e2.y >= e1.y) || 
					(e1.x >= e2.x && e2.x >= s1.x && e1.y >= e2.y && e2.y >= s1.y) || 
					(s1.x <= e2.x && e2.x <= e1.x && s1.y >= e2.y && e2.y >= e1.y) || 
					(e1.x <= e2.x && e2.x <= s1.x && e1.y >= e2.y && e2.y >= s1.y)){
					return e2;
				} else {			
					return Point(-1, -1);
				}
			} else {
				return Point(-1, -1);
			}
		}
	}
	float t = ((s2.y-e2.y)*(s1.x-s2.x) + (e2.x-s2.x)*(s1.y-s2.y)) / det;
	float u = ((s1.y-e1.y)*(s1.x-s2.x) + (e1.x-s1.x)*(s1.y-s2.y)) / det;

	Point p1 = Point(s1.x + t*(e1.x - s1.x), s1.y + t*(e1.y - s1.y));
	if (t > 0 && t < 1 && u > 0 && u < 1){
		return p1;
	}
	return Point(-1, -1);
}


bool get_intersection_segment_obstacles(Point s1, Point e1, std::vector<Polygon> obstacles){
	bool intersection = false;
	for (int obs = 0; obs < obstacles.size(); obs++){
		for (int vertex = 0; vertex < obstacles[obs].size(); vertex++){
			if (vertex != obstacles[obs].size() - 1){
				if (intersection_segment_segment(s1, e1, obstacles[obs][vertex], obstacles[obs][vertex + 1])){
					intersection = true;
					return intersection;
				}
			} else {
				if (intersection_segment_segment(s1, e1, obstacles[obs][vertex], obstacles[obs][0])){
					intersection = true;
					return intersection;
				}
			}
		}
	}
	return intersection;
}

