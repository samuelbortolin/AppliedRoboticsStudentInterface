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


bool arc_arc_coll(Point a1, double r1, double s1, double e1, Point a2, double r2, double s2, double e2){
	return false;
}

