#include "dubins.hpp"


float sinc(float t){
	float s;
	if (std::abs(t) < 0.002){  // For small values of t use Taylor series approximation
		s = 1 - std::pow(t,2) / 6.0 * (1 - std::pow(t,2) / 20.0);
	} else {
		s = std::sin(t) / t;
	}
	return s;
}


RobotBasePose compute_final_pose(RobotBasePose pos_i, float L, float k){
	RobotBasePose posf;
	posf.x = pos_i.x + L * sinc(k * L / 2.0) * std::cos(pos_i.theta + k * L / 2);
	posf.y = pos_i.y + L * sinc(k * L / 2.0) * std::sin(pos_i.theta + k * L / 2);
	posf.theta = mod2Pi(pos_i.theta + k *  L);
	return posf;
}


ScaledParametersWithLambda scale_to_standard_form(RobotBasePose pos0, RobotBasePose posf, float Kmax){  // Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
	ScaledParametersWithLambda sp_lambda;
	float dx, dy, phi;
	dx = posf.x - pos0.x;
	dy = posf.y - pos0.y;
	phi = std::atan2(dy, dx);
	sp_lambda.lambda = std::hypot(dx, dy) / 2;
	sp_lambda.sp.sp_Kmax = Kmax * sp_lambda.lambda;
	sp_lambda.sp.sp_theta_i = mod2Pi(pos0.theta - phi);
	sp_lambda.sp.sp_theta_f = mod2Pi(posf.theta - phi);
	return sp_lambda;
}


OriginalLength scale_from_standard_form(float lambda, StandardLength sl){  // Scale the solution to the standard problem back to the original problem
	OriginalLength ol;
	ol.s1 = sl.sp_s1 * lambda;
	ol.s2 = sl.sp_s2 * lambda;
	ol.s3 = sl.sp_s3 * lambda;
	return ol;
}


PrimitiveResult LSL(ScaledParameters sp){
	PrimitiveResult primitive;
	float invK = 1 / sp.sp_Kmax;
	float C = std::cos(sp.sp_theta_f) - std::cos(sp.sp_theta_i);
	float S = 2 * sp.sp_Kmax + std::sin(sp.sp_theta_i) - std::sin(sp.sp_theta_f);
	float temp1 = std::atan2(C, S);
	primitive.sl.sp_s1 = invK * mod2Pi(temp1 - sp.sp_theta_i);
	float temp2 = 2 + 4 * std::pow(sp.sp_Kmax,2) - 2 * std::cos(sp.sp_theta_i - sp.sp_theta_f) + 4 * sp.sp_Kmax * (std::sin(sp.sp_theta_i) - std::sin(sp.sp_theta_f));
	if (temp2 < 0){
		primitive.ok = false;
		primitive.sl.sp_s1 = 0.0;
		primitive.sl.sp_s2 = 0.0;
		primitive.sl.sp_s3 = 0.0;
	} else {
		primitive.ok = true;
		primitive.sl.sp_s2 = invK * std::sqrt(temp2);
		primitive.sl.sp_s3 = invK * mod2Pi(sp.sp_theta_f - temp1);
	}
	primitive.sum_sl = primitive.sl.sp_s1 + primitive.sl.sp_s2 + primitive.sl.sp_s3;
	return primitive;
}


PrimitiveResult RSR(ScaledParameters sp){
	PrimitiveResult primitive;
	float invK = 1 / sp.sp_Kmax;
	float C = std::cos(sp.sp_theta_i) - std::cos(sp.sp_theta_f);
	float S = 2 * sp.sp_Kmax - std::sin(sp.sp_theta_i) + std::sin(sp.sp_theta_f);
	float temp1 = std::atan2(C, S);
	primitive.sl.sp_s1 = invK * mod2Pi(sp.sp_theta_i - temp1);
	float temp2 = 2 + 4 * std::pow(sp.sp_Kmax,2) - 2 * std::cos(sp.sp_theta_i - sp.sp_theta_f) - 4 * sp.sp_Kmax * (std::sin(sp.sp_theta_i) - std::sin(sp.sp_theta_f));
	if (temp2 < 0){
		primitive.ok = false;
		primitive.sl.sp_s1 = 0.0;
		primitive.sl.sp_s2 = 0.0;
		primitive.sl.sp_s3 = 0.0;
	} else {
		primitive.ok = true;
		primitive.sl.sp_s2 = invK * std::sqrt(temp2);
		primitive.sl.sp_s3 = invK * mod2Pi(temp1 - sp.sp_theta_f);
	}
	primitive.sum_sl = primitive.sl.sp_s1 + primitive.sl.sp_s2 + primitive.sl.sp_s3;
	return primitive;
}


PrimitiveResult LSR(ScaledParameters sp){
	PrimitiveResult primitive;
	float invK = 1 / sp.sp_Kmax;
	float C = std::cos(sp.sp_theta_f) + std::cos(sp.sp_theta_i);
	float S = 2 * sp.sp_Kmax + sin(sp.sp_theta_i) + sin(sp.sp_theta_f);
	float temp1 = std::atan2(-C, S);
	float temp3 = 4 * std::pow(sp.sp_Kmax,2) - 2 + 2 * std::cos(sp.sp_theta_i - sp.sp_theta_f) + 4 * sp.sp_Kmax * (std::sin(sp.sp_theta_i) + std::sin(sp.sp_theta_f));
	if (temp3 < 0){
		primitive.ok = false;
		primitive.sl.sp_s1 = 0.0;
		primitive.sl.sp_s2 = 0.0;
		primitive.sl.sp_s3 = 0.0;
	} else {
		primitive.ok = true;
		primitive.sl.sp_s2 = invK * std::sqrt(temp3);
		float temp2 = -std::atan2(-2, primitive.sl.sp_s2 * sp.sp_Kmax);
		primitive.sl.sp_s1 = invK * mod2Pi(temp1 + temp2 - sp.sp_theta_i);
		primitive.sl.sp_s3 = invK * mod2Pi(temp1 + temp2 - sp.sp_theta_f);
	}
	primitive.sum_sl = primitive.sl.sp_s1 + primitive.sl.sp_s2 + primitive.sl.sp_s3;
	return primitive;
}


PrimitiveResult RSL(ScaledParameters sp){
	PrimitiveResult primitive;
	float invK = 1 / sp.sp_Kmax;
	float C = std::cos(sp.sp_theta_f) + std::cos(sp.sp_theta_i);
	float S = 2 * sp.sp_Kmax - std::sin(sp.sp_theta_i) - std::sin(sp.sp_theta_f);
	float temp1 = std::atan2(C, S);
	float temp3 = 4 * std::pow(sp.sp_Kmax,2) - 2 + 2 * std::cos(sp.sp_theta_i - sp.sp_theta_f) - 4 * sp.sp_Kmax * (std::sin(sp.sp_theta_i) + std::sin(sp.sp_theta_f));
	if (temp3 < 0){
		primitive.ok = false;
		primitive.sl.sp_s1 = 0.0;
		primitive.sl.sp_s2 = 0.0;
		primitive.sl.sp_s3 = 0.0;
	} else {
		primitive.ok = true;
		primitive.sl.sp_s2 = invK * std::sqrt(temp3);
		float temp2 = std::atan2(2, primitive.sl.sp_s2 * sp.sp_Kmax);
		primitive.sl.sp_s1 = invK * mod2Pi(sp.sp_theta_i - temp1 + temp2);
		primitive.sl.sp_s3 = invK * mod2Pi(sp.sp_theta_f - temp1 + temp2);
	}
	primitive.sum_sl = primitive.sl.sp_s1 + primitive.sl.sp_s2 + primitive.sl.sp_s3;
	return primitive;
}


PrimitiveResult RLR(ScaledParameters sp){
	PrimitiveResult primitive;
	float invK = 1 / sp.sp_Kmax;
	float C = std::cos(sp.sp_theta_i) - std::cos(sp.sp_theta_f);
	float S = 2 * sp.sp_Kmax - std::sin(sp.sp_theta_i) + std::sin(sp.sp_theta_f);
	float temp1 = std::atan2(C, S);
	float temp2 = 0.125 * (6 - 4 * std::pow(sp.sp_Kmax,2) + 2 * std::cos(sp.sp_theta_i - sp.sp_theta_f) + 4 * sp.sp_Kmax * (std::sin(sp.sp_theta_i) - std::sin(sp.sp_theta_f)));
	if (std::abs(temp2) > 1){
		primitive.ok = false;
		primitive.sl.sp_s1 = 0.0;
		primitive.sl.sp_s2 = 0.0;
		primitive.sl.sp_s3 = 0.0;
	} else {
		primitive.ok = true;
		primitive.sl.sp_s2 = invK * mod2Pi(2 * M_PI - std::acos(temp2));
		primitive.sl.sp_s1 = invK * mod2Pi(sp.sp_theta_i - temp1 + 0.5 * primitive.sl.sp_s2 * sp.sp_Kmax);
		primitive.sl.sp_s3 = invK * mod2Pi(sp.sp_theta_i - sp.sp_theta_i + sp.sp_Kmax * (primitive.sl.sp_s2 - primitive.sl.sp_s1));
	}
	primitive.sum_sl = primitive.sl.sp_s1 + primitive.sl.sp_s2 + primitive.sl.sp_s3;
	return primitive;
}


PrimitiveResult LRL(ScaledParameters sp){
	PrimitiveResult primitive;
	float invK = 1 / sp.sp_Kmax;
	float C = std::cos(sp.sp_theta_f) - std::cos(sp.sp_theta_i);
	float S = 2 * sp.sp_Kmax + sin(sp.sp_theta_i) - std::sin(sp.sp_theta_f);
	float temp1 = std::atan2(C, S);
	float temp2 = 0.125 * (6 - 4 * std::pow(sp.sp_Kmax,2) + 2 * std::cos(sp.sp_theta_i - sp.sp_theta_f) - 4 * sp.sp_Kmax * (std::sin(sp.sp_theta_i) - std::sin(sp.sp_theta_f)));
	if (std::abs(temp2) > 1){
		primitive.ok = false;
		primitive.sl.sp_s1 = 0.0;
		primitive.sl.sp_s2 = 0.0;
		primitive.sl.sp_s3 = 0.0;
	} else {
		primitive.ok = true;
		primitive.sl.sp_s2 = invK * mod2Pi(2 * M_PI - std::acos(temp2));
		primitive.sl.sp_s1 = invK * mod2Pi(temp1 - sp.sp_theta_i + 0.5 * primitive.sl.sp_s2 * sp.sp_Kmax);
		primitive.sl.sp_s3 = invK * mod2Pi(sp.sp_theta_f - sp.sp_theta_i + sp.sp_Kmax * (primitive.sl.sp_s2 - primitive.sl.sp_s1));
	}
	primitive.sum_sl = primitive.sl.sp_s1 + primitive.sl.sp_s2 + primitive.sl.sp_s3;
	return primitive;
}


DubinsArc create_dubins_arc(RobotBasePose pos0, float k, float L){
	DubinsArc arc;
	arc.pos0 = pos0;
	arc.k = k;
	arc.L = L;
	arc.posf = compute_final_pose(pos0, L, k);
	return arc;
}


DubinsCurve create_dubins_curve(RobotBasePose pos0, OriginalLength ol, float *ks){
	DubinsCurve curve;
	curve.a1 = create_dubins_arc(pos0, ks[0], ol.s1);
	curve.a2 = create_dubins_arc(curve.a1.posf, ks[1], ol.s2);
	curve.a3 = create_dubins_arc(curve.a2.posf, ks[2], ol.s3);
	curve.L = ol.s1 + ol.s2 + ol.s3;
	return curve;
}


bool check_collision(DubinsArc a, std::vector<Polygon> obstacles_and_borders, float Kmax){
	std::vector<Point> line1;
	std::vector<Point> line2;
	double x, y, s, e;
	if (a.k == 0){
		line2 = {Point(a.pos0.x, a.pos0.y), Point(a.posf.x, a.posf.y)};
	} else {
		if (a.k > 0){
			x = a.pos0.x - std::sin(a.pos0.theta)/Kmax;
			y = a.pos0.y + std::cos(a.pos0.theta)/Kmax;
			s = mod2Pi(a.pos0.theta - M_PI/2);
			e = mod2Pi(a.posf.theta - M_PI/2);
		}
		if (a.k < 0){
			x = a.pos0.x + std::sin(a.pos0.theta)/Kmax;
			y = a.pos0.y - std::cos(a.pos0.theta)/Kmax;
			s = mod2Pi(a.posf.theta + M_PI/2);
			e = mod2Pi(a.pos0.theta + M_PI/2);
		}
	}
	for (int i = 0; i < obstacles_and_borders.size(); i++){
		for (int j = 0; j < obstacles_and_borders[i].size(); j++){
			if (j != obstacles_and_borders[i].size() - 1){
				line1 = {obstacles_and_borders[i][j], obstacles_and_borders[i][j+1]};
			} else {
				line1 = {obstacles_and_borders[i][j], obstacles_and_borders[i][0]};
			}
			if (a.k == 0){
				if (intersection_segment_segment(line1[0], line1[1], line2[0], line2[1])){
					return true;
				}
			} else {
				if (intersection_arc_segment(Point(x, y), 1/Kmax, s, e, line1[0], line1[1])){
					return true;
				}
			}
		}
	}
	return false;
}


void get_dubins_arc_points(DubinsArc arc, std::vector<DubinsPathPoint>& dubins_path_points, float ds){
	float s;
	if (dubins_path_points.empty()){
		s = 0.0;
	} else {
		s = dubins_path_points.back().s;
	}
	for (float l=0; l<arc.L; l+=ds){
		DubinsPathPoint dubins_path_point;
		dubins_path_point.pos = compute_final_pose(arc.pos0, std::min(l, arc.L), arc.k);
		dubins_path_point.s = s + std::min(l, arc.L);
		dubins_path_point.k = arc.k;
		dubins_path_points.push_back(dubins_path_point);
	}
}


std::vector<DubinsPathPoint> get_dubins_curve_points(DubinsCurve curve, float ds){
	std::vector<DubinsPathPoint> dubins_path_points;
	get_dubins_arc_points(curve.a1, dubins_path_points, ds);
	get_dubins_arc_points(curve.a2, dubins_path_points, ds);
	get_dubins_arc_points(curve.a3, dubins_path_points, ds);
	return dubins_path_points;
}


ShortestDubinsPath create_dubins_path(RobotBasePose pos0, RobotBasePose posf, float Kmax, std::vector<Polygon> obstacles_and_borders, float ds){
	ShortestDubinsPath shortest_dubins_path;
	if (pos0.x != posf.x || pos0.y != posf.y){
		ScaledParametersWithLambda sp_lambda = scale_to_standard_form(pos0, posf, Kmax);
		DubinsTypes primitives[] = {
			LSL,
			RSR,
			LSR,
			RSL,
			RLR,
			LRL
		};
		const int ksigns[6][3] = {
			 1,  0,  1,
			-1,  0, -1,
			 1,  0, -1,
			-1,  0,  1,
			-1,  1, -1,
			 1, -1,  1
		};
		PrimitiveResult pr_cur;
		StandardLength sl_best;
		std::vector<DubinsCurve> result;
		for (int i=0; i<6; i++){
			pr_cur = primitives[i](sp_lambda.sp);
			pr_cur.id = i;
			if (!pr_cur.ok){
				continue;
			}
			OriginalLength ol = scale_from_standard_form(sp_lambda.lambda, pr_cur.sl);
			float ks[3] = {
				ksigns[pr_cur.id][0] * Kmax,
				ksigns[pr_cur.id][1] * Kmax,
				ksigns[pr_cur.id][2] * Kmax
			};
			DubinsCurve dc = create_dubins_curve(pos0, ol, ks);
			if (result.size()==0){
				result.push_back(dc);
			} else if (result.size()==1){
				if (dc.L > result.back().L){
					result.push_back(dc);
				} else {
					result.insert(result.begin(), dc);
				}
			} else {
				if (dc.L < result[0].L){
					result.insert(result.begin(), dc);
				} else if (dc.L > result.back().L){
					result.push_back(dc);
				} else {
					for (int j=0; j<(result.size()-1); j++){
						if (dc.L >= result[j].L && dc.L <= result[j+1].L){
							result.insert(result.begin()+j+1, dc);
							break;
						}
					}
				}
			}
		}
		if (result.size() > 0){
			for (int i = 0; i < result.size(); i++){
				bool coll = false;
				if (result[i].a1.L > 0){
					coll = check_collision(result[i].a1, obstacles_and_borders, Kmax);
				}
				if (result[i].a2.L > 0 && !coll){
					coll = check_collision(result[i].a2, obstacles_and_borders, Kmax);
				}
				if (result[i].a3.L > 0 && !coll){
					coll = check_collision(result[i].a3, obstacles_and_borders, Kmax);
				}
				if (!coll){
					shortest_dubins_path.find_dubins = true;
					shortest_dubins_path.curve = result[i];
					shortest_dubins_path.dubins_path_points = get_dubins_curve_points(result[i], ds);
					break;
				}
			}
		}
	} else {
		shortest_dubins_path.find_dubins = true;
		DubinsCurve curve;
		DubinsArc a1;
		a1.pos0 = pos0;
		a1.k = 0.0;
		a1.L = 0.0;
		a1.posf = pos0;
		curve.a1 = a1;
		DubinsArc a2;
		a2.pos0 = pos0;
		a2.k = 0.0;
		a2.L = 0.0;
		a2.posf = pos0;
		curve.a2 = a2;
		DubinsArc a3;
		a3.pos0 = pos0;
		a3.k = 0.0;
		a3.L = 0.0;
		a3.posf = pos0;
		curve.a3 = a3;
		curve.L = 0.0;
		shortest_dubins_path.curve = curve;
	}
	return shortest_dubins_path;
}


std::vector<ShortestDubinsPath> find_multipoint_dubins_path(std::vector<RobotBasePose> path_points, std::vector<Polygon> obstacles_and_borders, float Kmax, float ds){
	int ntheta = 72;
	int npoint = path_points.size();
	float dp[npoint-1][ntheta];
	int dp_index[npoint-1][ntheta];
	float dtheta = 2*M_PI/ntheta;
	std::vector<ShortestDubinsPath> multipoint_dubins_path;
	ShortestDubinsPath shortest_dubins_path;
	if (npoint == 2){
		dp[0][0] = std::numeric_limits<float>::infinity();
		for (int i = 0; i < ntheta; i++){
			path_points[1].theta = i*dtheta;
			shortest_dubins_path = create_dubins_path(path_points[0], path_points[1], Kmax, obstacles_and_borders, ds);
			if (!shortest_dubins_path.find_dubins){
				continue;
			}

			if (shortest_dubins_path.curve.L < dp[0][0]){
				dp[0][0] = shortest_dubins_path.curve.L;
				dp_index[0][0] = i;
			}
		}
	} else {
		for (int i = npoint-2; i > 0; i--){
			for (int j = 0; j < ntheta; j++){
				dp[i][j] = std::numeric_limits<float>::infinity();
				path_points[i].theta = j*dtheta;
				if (path_points[i].x != path_points[i+1].x || path_points[i].y != path_points[i+1].y){
					for (int k = 0; k < ntheta; k++){
						path_points[i+1].theta = k*dtheta;
						shortest_dubins_path = create_dubins_path(path_points[i], path_points[i+1], Kmax, obstacles_and_borders, ds);
						if (!shortest_dubins_path.find_dubins){
							continue;
						}

						if (i == npoint-2){
							if (shortest_dubins_path.curve.L < dp[i][j]){
								dp[i][j] = shortest_dubins_path.curve.L;
								dp_index[i][j] = k;
							}
						} else {
							if ((shortest_dubins_path.curve.L + dp[i+1][k]) < dp[i][j]){
								dp[i][j] = shortest_dubins_path.curve.L + dp[i+1][k];
								dp_index[i][j] = k;
							}
						}
					}
				} else {
					dp[i][j] = dp[i+1][j];
					dp_index[i][j] = dp_index[i+1][j];
				}
			}
		}
		if (path_points[0].x != path_points[1].x || path_points[0].y != path_points[1].y){
			dp[0][0] = std::numeric_limits<float>::infinity();
			for (int i = 0; i < ntheta; i++){
				path_points[1].theta = i*dtheta;
				shortest_dubins_path = create_dubins_path(path_points[0], path_points[1], Kmax, obstacles_and_borders, ds);
				if (!shortest_dubins_path.find_dubins){
					continue;
				}

				if ((shortest_dubins_path.curve.L + dp[1][i]) < dp[0][0]){
					dp[0][0] = shortest_dubins_path.curve.L + dp[1][i];
					dp_index[0][0] = i;
				}
			}
		} else {
			int theta_idx = 0;
			float theta_diff = path_points[0].theta;
			for (int i = 1; i < ntheta; i++){
				if (theta_diff > std::abs(path_points[0].theta - i*dtheta)){
					theta_idx = i;
					theta_diff = std::abs(path_points[0].theta - i*dtheta);
				}
			}
			dp[0][0] = dp[1][theta_idx];
			dp_index[0][0] = dp_index[1][theta_idx];
		}
	}
	if (dp[0][0] < std::numeric_limits<float>::infinity()){
		int id;
		for (int i = 0; i < npoint-1; i++){
			if (i == 0){
				id = dp_index[0][0];
			} else {
				id = dp_index[i][id];
			}
			path_points[i+1].theta = id*dtheta;
			shortest_dubins_path = create_dubins_path(path_points[i], path_points[i+1], Kmax, obstacles_and_borders, ds);
			multipoint_dubins_path.emplace_back(shortest_dubins_path);
		}
	}
	return multipoint_dubins_path;
}

