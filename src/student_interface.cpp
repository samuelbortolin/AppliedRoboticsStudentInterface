#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <cmath>
#include <chrono>
#include <atomic>
#include <unistd.h>
#include <experimental/filesystem>
#include "offset.hpp"
#include "intersection.hpp"
#include "vertical_cell_decomposition.hpp"
#include "dubins.hpp"
#include "planning.hpp"

using namespace std::chrono;


namespace student {
	void loadImage(cv::Mat& img_out, const std::string& config_folder){
		throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT LOADED" );
	}

	void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
		static size_t id = 0;
		static bool init = false;
		static std::string folder_path;

		if(!init){
			bool exist = true;
			int i = 0;
			while(exist && i < 1000){
				std::stringstream ss;
				ss << config_folder << "/camera_image" << std::setw(3) << std::setfill('0') << i << "/";
 				folder_path = ss.str();
				exist = std::experimental::filesystem::exists(folder_path);
				i++;
			}

			if(i > 999 || !std::experimental::filesystem::create_directories(folder_path)){
				throw std::logic_error( "NO EMTY FOLDER" );
			}

			init = true;
		}

		cv::imshow(topic, img_in);
		char c;
		c = cv::waitKey(30);
		std::stringstream img_file;
		switch(c){
			case 's':
				img_file << folder_path << std::setfill('0') << std::setw(3)  << (id++) << ".jpg";
				cv::imwrite( img_file.str(), img_in );

				std::cout << "Saved image " << img_file.str() << std::endl;
				break;
			default:
				break;
		}
	}

	// Defintion of the function pickNPoints and the callback mouseCallback.
	// The function pickNPoints is used to display a window with a background
	// image, and to prompt the user to select n points on this image.
	static cv::Mat bg_img;
	static std::vector<cv::Point2f> result;
	static std::string name;
	static std::atomic<bool> done;
	static int n;
	static double show_scale = 1.0;

	void mouseCallback(int event, int x, int y, int, void* p){
		if(event != cv::EVENT_LBUTTONDOWN || done.load()) return;

		result.emplace_back(x*show_scale, y*show_scale);
		cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
		cv::imshow(name.c_str(), bg_img);

		if(result.size() >= n){
			usleep(500*1000);
			done.store(true);
		}
	}

	std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img){
		result.clear();
		cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
		cv::resize(img, bg_img, small_size);
		// bg_img = img.clone();
		name = "Pick " + std::to_string(n0) + " points";
		cv::imshow(name.c_str(), bg_img);
		cv::namedWindow(name.c_str());
		n = n0;

		done.store(false);

		cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
		while(!done.load()){
			cv::waitKey(500);
		}

		cv::destroyWindow(name.c_str());
		return result;
	}

	bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
		std::string file_path = config_folder + "/extrinsicCalib.csv";
		std::vector<cv::Point2f> image_points;
		if(!std::experimental::filesystem::exists(file_path)){
			std::experimental::filesystem::create_directories(config_folder);
			image_points = pickNPoints(4, img_in);
			// SAVE POINT TO FILE
			// std::cout << "IMAGE POINTS: " << std::endl;
 			// for (const auto pt: image_points) {
			//   std::cout << pt << std::endl;
			// }
			std::ofstream output(file_path);
			if(!output.is_open()){
				throw std::runtime_error("Cannot write file: " + file_path);
			}
			for(const auto pt: image_points){
				output << pt.x << " " << pt.y << std::endl;
			}
			output.close();
		}else{
			// LOAD POINT FROM FILE
			std::ifstream input(file_path);
			if (!input.is_open()){
				throw std::runtime_error("Cannot read file: " + file_path);
			}
			while(!input.eof()){
				double x, y;
				if(!(input >> x >> y)){
					if (input.eof()) break;
					else{
						throw std::runtime_error("Malformed file: " + file_path);
					}
				}
				image_points.emplace_back(x, y);
			}
			input.close();
		}

		cv::Mat dist_coeffs;
		dist_coeffs = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
		bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

		// cv::Mat Rt;
		// cv::Rodrigues(rvec_, Rt);
		// auto R = Rt.t();
		// auto pos = -R * tvec_;

		if(!ok){
			std::cerr << "FAILED SOLVE_PNP" << std::endl;
		}

		return ok;
	}

	void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
		// SLOW VERSION
		// undistort(img_in, img_out, cam_matrix, dist_coeffs);

		// OPTIMIZED VERSION
		static bool maps_initialized = false;
		static cv::Mat full_map1, full_map2;

		if(!maps_initialized){
			// Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
			cv::Mat R;
			cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_in.size(), CV_16SC2, full_map1, full_map2);

			maps_initialized = true;
		}

		// Initialize output image
		cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);
	}

	void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, const std::vector<cv::Point2f>& dest_image_points_plane, cv::Mat& plane_transf, const std::string& config_folder){
		cv::Mat image_points;

		// project points
		cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

		plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
	}

	void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const std::string& config_folder){
		cv::warpPerspective(img_in, img_out, transf, img_in.size());
	}

	bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){
		// Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
		cv::Mat red_mask_low, red_mask_high, red_mask;
		cv::inRange(hsv_img, cv::Scalar(0, 40, 40), cv::Scalar(15, 255, 255), red_mask_low);
		cv::inRange(hsv_img, cv::Scalar(163, 40, 40), cv::Scalar(180, 255, 255), red_mask_high);
		cv::add(red_mask_low, red_mask_high, red_mask);

		// cv::Mat img_small;
		// cv::resize(red_mask, img_small, cv::Size(640, 512));
		// cv::imshow("obstacles", red_mask);
		// cv::waitKey(1);

		std::vector<std::vector<cv::Point>> contours, contours_approx;
		std::vector<cv::Point> approx_curve;
		cv::Mat contours_img;

		// Process red mask
		// contours_img = img_in.clone();
		cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		// drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
		// std::cout << "N. contours: " << contours.size() << std::endl;
		for(int i=0; i<contours.size(); ++i){
			//std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
			approxPolyDP(contours[i], approx_curve, 3, true);

			Polygon scaled_contour;
			for (const auto& pt: approx_curve) {
				scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
			}
			obstacle_list.push_back(scaled_contour);
			// contours_approx = {approx_curve};
			// drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
			// std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
		}
		// std::cout << std::endl;
		// cv::imshow("Original", contours_img);
		// cv::waitKey(1);

		return true;
	}

	bool processGate(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& gate_list){
		// Find purple regions
		cv::Mat purple_mask;
		cv::inRange(hsv_img, cv::Scalar(130, 10, 10), cv::Scalar(165, 255, 255), purple_mask);

		std::vector<std::vector<cv::Point>> contours, contours_approx;
		std::vector<cv::Point> approx_curve;
		// cv::Mat contours_img;

		// Process purple mask
		// contours_img = hsv_img.clone();
		cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		// drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
		// std::cout << "N. contours: " << contours.size() << std::endl;

		bool res = false;
		for(auto& contour: contours){
			const double area = cv::contourArea(contour);
			// std::cout << "AREA " << area << std::endl;
			// std::cout << "SIZE: " << contours.size() << std::endl;
			if(area > 500){
				approxPolyDP(contour, approx_curve, 30, true);

				if(approx_curve.size()!=4) continue;

				// contours_approx = {approx_curve};
				// drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

                Polygon gate;
				for(const auto& pt: approx_curve){
					gate.emplace_back(pt.x/scale, pt.y/scale);
				}
				gate_list.push_back(gate);
				res = true;
			}
		}

		// cv::imshow("Original", contours_img);
		// cv::waitKey(1);

		return res;
	}

	bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<Polygon>& gate_list, const std::string& config_folder){
		// Convert color space from BGR to HSV
		cv::Mat hsv_img;
		cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

		const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
		if(!res1) std::cout << "processObstacles return false" << std::endl;

		const bool res2 = processGate(hsv_img, scale, gate_list);
		if(!res2) std::cout << "processGate return false" << std::endl;

		return res1 && res2;
	}

	bool processRobot(const cv::Mat& hsv_img, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string ns){
		cv::Mat robot_mask;
		if (ns == "/my_robot_0") {
			// Robot 0 blue
			cv::inRange(hsv_img, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), robot_mask);
		}else if (ns == "/my_robot_1"){
			// Robot 1 yellow
			cv::inRange(hsv_img, cv::Scalar(15, 0, 0), cv::Scalar(36, 255, 255), robot_mask);
		}else if (ns == "/my_robot_2"){
			// Robot 2 green
			cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), robot_mask);
		}

		// Process masks
		std::vector<std::vector<cv::Point>> contours, contours_approx;
		std::vector<cv::Point> approx_curve;
		cv::findContours(robot_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		// cv::imshow("filterrrr"+ns, robot_mask);
		// cv::waitKey(1);
		// cv::Mat contours_img;
		// contours_img = hsv_img.clone();
		// drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
		// std::cout << "N. contours: " << contours.size() << std::endl;

		bool found = false;
		for (int i=0; i<contours.size(); ++i){
			// std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;

			cv::approxPolyDP(contours[i], approx_curve, 10, true);
			contours_approx = {approx_curve};

			// cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

			double area = cv::contourArea(approx_curve);

			if(approx_curve.size() != 3) continue;

			if(area < 300 || area>3000) continue;

			found = true;
			break;
		}

		if(found){
			for(const auto& pt: approx_curve){
				triangle.emplace_back(pt.x/scale, pt.y/scale);
			}

			double cx = 0, cy = 0;
			for(auto item: triangle){
				cx += item.x;
				cy += item.y;
			}
			cx /= triangle.size();
			cy /= triangle.size();

			double dst = 0;
			Point vertex;
			for(auto& item: triangle){
				double dx = item.x-cx;
				double dy = item.y-cy;
				double curr_d = dx*dx + dy*dy;
				if(curr_d > dst){
					dst = curr_d;
					vertex = item;
				}
			}

			// cv::Moments m = cv::moments(approx_curve, false);
			// cv::Point center(m.m10/m.m00, m.m01/m.m00);
			// cv::Vec4f line;
			// cv::fitLine(approx_curve, line, cv::DIST_L2, 0, 0.01, 0.01);
			// cv::line(warpedFrame, cv::Point(line[2], line[3]), cv::Point(line[2]+line(0)*80, line(3)+line(1)*80), (0,255,0), 2);


			// cv::line(contours_img, center*scale, vertex*scale, (0,255,0), 2);
			// cv::circle(contours_img, center*scale, 20, cv::Scalar(0,0,0), -1);

			double dx = cx-vertex.x;
			double dy = cy-vertex.y;

			x = cx;
			y = cy;
			theta = std::atan2(dy, dx);

			// covariance = {};

			// std::cout << xc_m << " " << yc_m << " " << theta*180/M_PI << std::endl;
		}

		// cv::imshow("Original", contours_img);
		// cv::waitKey(1);

		return found;
	}

	bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string ns, const std::string& config_folder){
		// Convert color space from BGR to HSV
		cv::Mat hsv_img;
		cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
		return processRobot(hsv_img, scale, triangle, x, y, theta, ns);
	}

	bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<Polygon>& gate_list, const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta, std::vector<Path>& path, const std::string& config_folder){
		auto start = high_resolution_clock::now();
		// Parameters
		float offset_value = 0.06;		// The offset value
		float maximum_curvature = 15.0;		// The maximum curvature value
		float ds = 0.03;			// The curvilinear abscissa of the movement of the robot from one point to the following one in reaching the next node
		bool synchronous_movement = true;	// Whether to move synchronously the robots, in that case the robots will just move forward of one step in the graph

		bool debug_logs = false;
		bool show_plots = false;
		int wait_key_time = 6000;
		bool store_plots = false;
		std::string image_folder = "/home/ubuntu/workspace/project/images/";
		cv::Mat plot(1100, 1600, CV_8UC3, cv::Scalar(255, 255, 255));

		std::cout << std::endl << std::endl << " ------ Starting planning evaquation: ------ " << std::endl << std::endl;

		if (debug_logs){
			std::cout << std::endl << " --- Robots positions: --- " << std::endl;
			for (int i = 0; i < x.size(); i++){
				std::cout << x[i] << " " << y[i] << std::endl;
			}

			std::cout << std::endl << " --- Gates position: --- " << std::endl;
			for (int i = 0; i < gate_list.size(); i++){
				std::cout << get_cell_centroid(gate_list[i]).x << " " << get_cell_centroid(gate_list[i]).y << std::endl;
			}

			std::cout << std::endl << " --- Borders: --- " << std::endl;
			for (Point point : borders){
				std::cout << point.x << " " << point.y << std::endl;
			}

			std::cout << std::endl << " --- Obstacles: --- " << std::endl;
			for (Polygon obstacle : obstacle_list){
				for (Point point : obstacle){
					std::cout << point.x << " " << point.y << std::endl;
				}
				std::cout << std::endl;
			}
		}

		// Add offset the borders of the arena
		const Polygon borders_with_offset = add_offset_to_borders(borders, -offset_value);
		if (debug_logs){
			std::cout << std::endl << " --- Borders with offset: --- " << std::endl;
		}
		for (int i=0; i<borders_with_offset.size(); ++i){
			if (debug_logs){
				std::cout << borders_with_offset[i].x << " " << borders_with_offset[i].y << std::endl;
			}
			if (show_plots or store_plots){
				if (i != borders_with_offset.size() - 1){
					cv::line(plot, cv::Point2f(borders_with_offset[i].x*1000, borders_with_offset[i].y*1000), cv::Point2f(borders_with_offset[i + 1].x*1000, borders_with_offset[i + 1].y*1000), cv::Scalar(255, 0, 0), 2);
				} else {
					cv::line(plot, cv::Point2f(borders_with_offset[i].x*1000, borders_with_offset[i].y*1000), cv::Point2f(borders_with_offset[0].x*1000, borders_with_offset[0].y*1000), cv::Scalar(255, 0, 0), 2);
				}
			}
		}

		// Add offset the obstacles in the arena
		std::vector<Polygon> obstacle_list_with_offset = add_offset_to_obstacles(obstacle_list, offset_value);
		if (debug_logs){
			std::cout << std::endl << " --- Obstacles with offset: --- " << std::endl;
			for (Polygon obstacle : obstacle_list_with_offset){
				for (int i=0; i<obstacle.size(); ++i){
					std::cout << obstacle[i].x << " " << obstacle[i].y << std::endl;
				}
				std::cout << std::endl;
			}
		}

		// Merge overlapping obstacles and consider borders
		std::vector<Polygon> merged_obstacles = merge_obstacles(obstacle_list_with_offset, borders_with_offset);
		if (debug_logs){
			std::cout << std::endl << " --- Merged obstacles: --- " << std::endl;
			for (Polygon obstacle : merged_obstacles){
				for (int i=0; i<obstacle.size(); ++i){
					std::cout << obstacle[i].x << " " << obstacle[i].y << std::endl;
				}
				std::cout << std::endl;
			}
		}

		// Then create the convex hull to get the final obstacles
		std::vector<Polygon> convex_hull_merged_obstacles = create_convex_hull(merged_obstacles);
		if (debug_logs){
			std::cout << std::endl << " --- Convex hull merged obstacles: --- " << std::endl;
		}
		for (Polygon obstacle : convex_hull_merged_obstacles){
			for (int i=0; i<obstacle.size(); ++i){
				if (debug_logs){
					std::cout << obstacle[i].x << " " << obstacle[i].y << std::endl;
				}
				if (show_plots or store_plots){
					if (i != obstacle.size() - 1){
						cv::line(plot, cv::Point2f(obstacle[i].x*1000, obstacle[i].y*1000), cv::Point2f(obstacle[i + 1].x*1000, obstacle[i + 1].y*1000), cv::Scalar(255, 0, 0), 2);
					} else {
						cv::line(plot, cv::Point2f(obstacle[i].x*1000, obstacle[i].y*1000), cv::Point2f(obstacle[0].x*1000, obstacle[0].y*1000), cv::Scalar(255, 0, 0), 2);
					}
				}
			}
			if (debug_logs){
				std::cout << std::endl;
			}
		}

		std::vector<Polygon> obstacles_and_borders;
		for (int i=0; i<convex_hull_merged_obstacles.size(); i++){
			obstacles_and_borders.push_back(convex_hull_merged_obstacles[i]);
		}
		obstacles_and_borders.push_back(borders_with_offset);

		if (show_plots){
			cv::imshow("Inflated", plot);
			cv::waitKey(wait_key_time);
		}
		if (store_plots){
			cv::imwrite(image_folder + "Inflated.png", plot);
		}

		// Sort vertices
		auto start_roadmap = high_resolution_clock::now();
		std::vector <std::tuple<Point, int> > sorted_vertices = sort_vertices(convex_hull_merged_obstacles);
		if (debug_logs){
			std::cout << std::endl << " --- Sorted vertices: --- " << std::endl;
			for (std::tuple<Point, int> vertex : sorted_vertices){
				std::cout << std::get<0>(vertex).x << " " << std::get<0>(vertex).y << std::endl;
			}
		}

		// Find VCD cells
		std::vector<Polygon> cells = find_cells(borders_with_offset, sorted_vertices, convex_hull_merged_obstacles, offset_value);
		if (debug_logs){
			std::cout << std::endl << " --- Cells: --- " << std::endl;
		}
		for (Polygon cell : cells){
			for (int i=0; i<cell.size(); ++i){
				if (debug_logs){
					std::cout << cell[i].x << " " << cell[i].y << std::endl;
				}
				if (show_plots or store_plots){
					if (i != cell.size() - 1){
						cv::line(plot, cv::Point2f(cell[i].x*1000, cell[i].y*1000), cv::Point2f(cell[i + 1].x*1000, cell[i + 1].y*1000), cv::Scalar(0, 255, 0), 2);
					} else {
						cv::line(plot, cv::Point2f(cell[i].x*1000, cell[i].y*1000), cv::Point2f(cell[0].x*1000, cell[0].y*1000), cv::Scalar(0, 255, 0), 2);
					}
				}
			}
			if (debug_logs){
				std::cout << std::endl;
			}
			if (show_plots or store_plots){
				cv::circle(plot, cv::Point2f(get_cell_centroid(cell).x*1000, get_cell_centroid(cell).y*1000), 2, cv::Scalar(0, 0, 255), 2);
			}
		}

		if (show_plots){
			cv::imshow("VCD", plot);
			cv::waitKey(wait_key_time);
		}
		if (store_plots){
			cv::imwrite(image_folder + "VCD.png", plot);
		}

		// Get roadmap from cells
		std::tuple< std::vector<Point>, std::vector< std::vector<float> > > roadmap = create_roadmap(cells, obstacles_and_borders, gate_list, x, y);
		auto stop_roadmap = high_resolution_clock::now();
		std::vector<Point> nodes = std::get<0>(roadmap);
		if (debug_logs){
			std::cout << std::endl << " --- Nodes: --- " << std::endl;
		}
		for (const Point& node : nodes){
			if (debug_logs){
				std::cout << node.x << " " << node.y << std::endl;
			}
			if (show_plots or store_plots){
				cv::circle(plot, cv::Point2f(node.x*1000, node.y*1000), 2, cv::Scalar(255, 0, 255), 2);
			}
		}

		std::vector< std::vector<float> > adjacency_matrix = std::get<1>(roadmap);
		if (debug_logs){
			std::cout << std::endl << " --- Adjacency matrix: --- " << std::endl;
		}
		for (int i=0; i<adjacency_matrix.size(); ++i){
			for (int j=0; j<adjacency_matrix[i].size(); ++j){
				if (debug_logs){
					std::cout << adjacency_matrix[i][j] << "\t";
				}
				if ((show_plots or store_plots) && adjacency_matrix[i][j] > 0.0){
					cv::line(plot, cv::Point2f(nodes[i].x*1000, nodes[i].y*1000), cv::Point2f(nodes[j].x*1000, nodes[j].y*1000), cv::Scalar(0, 255, 255), 2);
				}
			}
			if (debug_logs){
				std::cout << std::endl;
			}
		}

		// Using a UCS find the best feasibile path for all robots
		int target_node = adjacency_matrix.size() - 1;
		std::vector<float> optimal_cost = ucs(adjacency_matrix, target_node);
		std::vector<int> initial_nodes = {};
		for(int i=0; i<x.size(); i++){
			initial_nodes.push_back(adjacency_matrix.size() - gate_list.size() - x.size() + i);
		}

		if (debug_logs){
			std::cout << std::endl << " --- UCS optimal costs: --- " << std::endl;
		}
		for(int i=0; i<nodes.size(); i++){
			if (debug_logs){
				std::cout << optimal_cost[i] << std::endl;
			}
			if (show_plots or store_plots){
				if (i == target_node){
					cv::putText(plot,
						"target",
						cv::Point2f(nodes[i].x*1000, nodes[i].y*1000),
						cv::FONT_HERSHEY_DUPLEX,
						0.75,
						CV_RGB(255, 0, 255),
						2);
				} else if (std::find(initial_nodes.begin(), initial_nodes.end(), i) != initial_nodes.end()){
					cv::putText(plot,
						"robot " + std::to_string(optimal_cost[i]),
						cv::Point2f(nodes[i].x*1000, nodes[i].y*1000),
						cv::FONT_HERSHEY_DUPLEX,
						0.75,
						CV_RGB(255, 0, 255),
						2);
				} else {
					cv::putText(plot,
						std::to_string(optimal_cost[i]),
						cv::Point2f(nodes[i].x*1000, nodes[i].y*1000),
						cv::FONT_HERSHEY_DUPLEX,
						0.5,
						CV_RGB(255, 0, 255),
						2);
				}
			}
		}


		if (show_plots){
			cv::imshow("Roadmap", plot);
			cv::waitKey(wait_key_time);
		}
		if (store_plots){
			cv::imwrite(image_folder + "Roadmap.png", plot);
		}

		// Find optimal paths for all the robots without intersections
		std::vector<int> reachable_initial_nodes = {};
		for (int i=0; i<initial_nodes.size(); i++){
			if (optimal_cost[initial_nodes[i]] < 2 * offset_value){
				reachable_initial_nodes.push_back(target_node);
			} else {
				reachable_initial_nodes.push_back(initial_nodes[i]);
			}
		}

		std::vector<std::vector<int>> optimal_paths = find_optimal_paths(optimal_cost, nodes, adjacency_matrix, reachable_initial_nodes, target_node, 2 * offset_value);
		if (debug_logs){
			std::cout << std::endl << " --- Optimal paths: --- " << std::endl;
		}
		for(int i=0; i<optimal_paths.size(); i++){
			for(int j=0; j<optimal_paths[i].size(); j++){
				if (debug_logs){
					std::cout << optimal_paths[i][j] << " ";
				}
				if ((show_plots or store_plots) && j != optimal_paths[i].size() - 1){
					cv::line(plot, cv::Point2f(nodes[optimal_paths[i][j]].x*1000, nodes[optimal_paths[i][j]].y*1000), cv::Point2f(nodes[optimal_paths[i][j + 1]].x*1000, nodes[optimal_paths[i][j + 1]].y*1000), cv::Scalar(255, 255, 0), 5);
				}
			}
			if (debug_logs){
				std::cout << std::endl;
			}
		}

		if (show_plots){
			cv::imshow("BestPaths", plot);
			cv::waitKey(wait_key_time);
		}
		if (store_plots){
			cv::imwrite(image_folder + "BestPaths.png", plot);
		}

		// use multi-point dubins to smooth the paths and reach the target optimally
		std::vector< std::vector<RobotBasePose> > path_points = {};
		for (int i=0; i<optimal_paths.size(); i++){
			std::vector<RobotBasePose> one_path_points;
			RobotBasePose temp_pt;
			for (int j=0; j<optimal_paths[i].size(); j++) {
				temp_pt = {nodes[optimal_paths[i][j]].x, nodes[optimal_paths[i][j]].y, -1};
				one_path_points.push_back(temp_pt);
			}
			path_points.push_back(one_path_points);
			path_points[i][0].theta = mod2Pi(theta[i]);
		}

		if (debug_logs){
			std::cout << std::endl << " --- Multi-point Dubins: --- " << std::endl;
		}
		for(int robot = 0; robot < optimal_paths.size(); robot ++){
			if (path_points[robot].size() > 1){
				std::vector<ShortestDubinsPath> multipoint_dubins_path = find_multipoint_dubins_path(path_points[robot], obstacles_and_borders, maximum_curvature, ds);
				if (multipoint_dubins_path.size() > 0){
					int steps_to_perform;
					if (synchronous_movement){
						steps_to_perform = 1;
					} else {
						steps_to_perform = multipoint_dubins_path.size();
					}
					std::cout << "Found a Dubins path for robot " << robot + 1 << "!" << std::endl;
					for (int i = 0; i < steps_to_perform; i++){
						for (DubinsPathPoint dubins_path_point : multipoint_dubins_path[i].dubins_path_points){
							path[robot].points.emplace_back(dubins_path_point.s, dubins_path_point.pos.x, dubins_path_point.pos.y, dubins_path_point.pos.theta, dubins_path_point.k);
							if (debug_logs){
								std::cout << dubins_path_point.s << " " << dubins_path_point.pos.x << " " << dubins_path_point.pos.y << " " << dubins_path_point.pos.theta << " " << dubins_path_point.k << std::endl;
							}
						}
						if (debug_logs){
							std::cout << std::endl;
						}
					}
				} else {
					std::cout << "It is not possible to find a Dubins path for robot " << robot + 1 << "!" << std::endl;
				}
			} else {
				if (optimal_cost[initial_nodes[robot]] < 0){
					std::cout << "Robot " << robot + 1 << " can't reach the gate!" << std::endl;
				} else {
					std::cout << "Robot " << robot + 1 << " is already at the gate!" << std::endl;
				}
			}
		}

		if (show_plots){
			cv::destroyAllWindows();
		}

		auto stop = high_resolution_clock::now();

		auto duration_roadmap = duration_cast<microseconds>(stop_roadmap - start_roadmap);
		std::cout << "Time taken by roadmap: " << duration_roadmap.count() << " microseconds" << std::endl;

		auto duration = duration_cast<microseconds>(stop - start);
		std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;

		return true;
	}
}

