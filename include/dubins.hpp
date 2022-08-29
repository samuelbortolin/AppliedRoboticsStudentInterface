#ifndef DUBINS_HPP
#define DUBINS_HPP


#include "utils.hpp"
#include "intersection.hpp"
#include <limits>


struct RobotBasePose{
	float x;
	float y;
	float theta;
};


struct DubinsArc{
	RobotBasePose pos0;
	RobotBasePose posf;
	float k;
	float L;
};


struct DubinsCurve{
	DubinsArc arc1;
	DubinsArc arc2;
	DubinsArc arc3;
	float L;  // Total length of the curve
};


struct ScaledParameters{
	float sp_theta_i;
	float sp_theta_f;
	float sp_Kmax;
};


struct ScaledParametersWithLambda{
	ScaledParameters sp;
	float lambda;
};


struct StandardLength{
	float sp_s1;
	float sp_s2;
	float sp_s3;
};


struct OriginalLength{
	float s1;
	float s2;
	float s3;
};


struct PrimitiveResult{
	bool ok;
	StandardLength sl;
	float sum_sl;
	int id;
};


typedef PrimitiveResult (*DubinsTypes) (ScaledParameters sp);


struct DubinsPathPoint{
	RobotBasePose pos;
	float s;
	float k;
};


struct ShortestDubinsPath{
	bool find_dubins = false;
	DubinsCurve curve;
	std::vector<DubinsPathPoint> dubins_path_points;
};


/*!
* Find a multipoint dubins path for a given robot given the path on the graph.
* @param[in]  path_points			The points of path on the graph.
* @param[in]  obstacles_and_borders		The obstacles and borders.
* @param[in]  Kmax				The maximum curvature value.
* @param[in]  ds				The curvilinear abscissa of the movement of the robot from one point to the following one in reaching the next node.
*/
std::vector<ShortestDubinsPath> find_multipoint_dubins_path(std::vector<RobotBasePose> path_points, std::vector<Polygon> obstacles_and_borders, float Kmax, float ds);


#endif

