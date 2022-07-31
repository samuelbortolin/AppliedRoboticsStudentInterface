#include "intersection.hpp"


bool intersection_segment_segment(Point s1, Point e1, Point s2, Point e2){
   	return false;
}


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

//function to decide an angle within an arc (not used)
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

float abs(float value){
	if (value < 0) return value * (-1);
	return value;
}

// Normalize an angle in range [0,2*pi)
float mod2Pi(float angle)
{
    return angle - 2 * M_PI * floor(angle / (2 * M_PI));
}

bool intersection_arc_arc(Point a1, double r1, double s1, double e1, Point a2, double r2, double s2, double e2){
	double eucl_distance =sqrt(pow(a1.x-a2.x,2) + pow(a1.y-a2.y,2));
	if (eucl_distance <= (r1+r2) and eucl_distance >= abs(r1-r2)){
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
		}
		else{
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


























