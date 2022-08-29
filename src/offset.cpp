#include "offset.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/polygon.hpp"
#include "boost/geometry/geometries/adapted/boost_tuple.hpp"
#include "boost/geometry/algorithms/convex_hull.hpp"


BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)  // Register the Cartesian coordinate system


Polygon add_offset_to_borders(const Polygon& borders, float offset_value){
	Polygon borders_with_offset;
	ClipperLib::Path clipper_borders;
	ClipperLib::Paths clipper_borders_with_offset;

	// pass from borders to a clipper object
	for (const auto &position : borders) {
		clipper_borders << ClipperLib::IntPoint(position.x * 1000.0, position.y * 1000.0);
	}

	// apply the offset
	ClipperLib::ClipperOffset co;
	co.AddPath(clipper_borders, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
	co.Execute(clipper_borders_with_offset, offset_value * 1000.0);

	// pass from clipper object to borders
	for(const ClipperLib::Path &path : clipper_borders_with_offset){
		for(const ClipperLib::IntPoint &point: path){
			double x = point.X / 1000.0;
			double y = point.Y / 1000.0;
			borders_with_offset.emplace_back(x, y);
		}
	}
	return borders_with_offset;
}


std::vector<Polygon> add_offset_to_obstacles(const std::vector<Polygon>& obstacle_list, float offset_value){
	std::vector<Polygon> obstacle_list_with_offset;
	for (const Polygon obstacle : obstacle_list){
		ClipperLib::Path clipper_obstacle;
		ClipperLib::Paths clipper_obstacle_with_offset;

		// pass from obstacle to a clipper object
		for (const auto &position : obstacle){
			clipper_obstacle << ClipperLib::IntPoint(position.x * 1000.0, position.y * 1000.0);
		}

		// apply the offset
		ClipperLib::ClipperOffset co;
		co.AddPath(clipper_obstacle, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		co.Execute(clipper_obstacle_with_offset, offset_value * 1000.0);

		// pass from clipper object to a obstacle
		for (const ClipperLib::Path &path : clipper_obstacle_with_offset){
			Polygon polygon_with_offset;
			for (const ClipperLib::IntPoint &point: path){
				double x = point.X / 1000.0;
				double y = point.Y / 1000.0;
				polygon_with_offset.emplace_back(x, y);
			}
			obstacle_list_with_offset.emplace_back(polygon_with_offset);
		}
	}
	return obstacle_list_with_offset;
}


std::vector<Polygon> merge_obstacles(const std::vector<Polygon> &obstacle_list, const Polygon& borders){
	// pass from obstacles to clipper objects
	ClipperLib::Paths clipper_obstacles;
	for (const Polygon obstacle : obstacle_list){
		ClipperLib::Path clipper_obstacle;
		for (const auto &position : obstacle){
			clipper_obstacle.push_back(ClipperLib::IntPoint(position.x * 1000.0, position.y * 1000.0));
		}
		clipper_obstacles.push_back(clipper_obstacle);
	}

	// find the union of the obstacles
	ClipperLib::Clipper c_union;
	ClipperLib::Paths clipper_merged_obstacles;
	c_union.AddPaths(clipper_obstacles, ClipperLib::ptSubject, true);
	c_union.Execute(ClipperLib::ctUnion, clipper_merged_obstacles, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

	ClipperLib::Clipper c_intersection;
	ClipperLib::Path clipper_borders;
	ClipperLib::Paths clipper_merged_obstacles_considering_obstacles;

	// pass from borders to a clipper object
	for (const auto &position : borders) {
		clipper_borders << ClipperLib::IntPoint(position.x * 1000.0, position.y * 1000.0);
	}

	// find the intersection of the borders with the merged obstacles
	c_intersection.AddPath(clipper_borders, ClipperLib::ptSubject, true);
	c_intersection.AddPaths(clipper_merged_obstacles, ClipperLib::ptClip, true);
	c_intersection.Execute(ClipperLib::ctIntersection, clipper_merged_obstacles_considering_obstacles, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

	// convert merged clipper objects to merged obstacles
	std::vector<Polygon> merged_obstacles;
	for (const ClipperLib::Path &path : clipper_merged_obstacles_considering_obstacles){
		// for each point in the path
		Polygon merged_obstacle;
		for (const ClipperLib::IntPoint &point: path){
			double x = point.X / 1000.0;
			double y = point.Y / 1000.0;
			merged_obstacle.emplace_back(x, y);
		}
		merged_obstacles.push_back(merged_obstacle);
	}
	return merged_obstacles;
}


boost::geometry::model::polygon<boost::tuple<double, double>> convert_polygon_to_boost_polygon(const Polygon &polygon){
	std::string wkt_string = "polygon((";
	for (int i = 0; i < polygon.size(); i++){
		// Add the vertex segment pair to the string
		wkt_string += std::to_string(polygon[i].x) + " " + std::to_string(polygon[i].y) + ",";
	}
	// Add the closing vertex segment
	wkt_string += std::to_string(polygon[0].x) + " " + std::to_string(polygon[0].y);
	wkt_string += "))";
	boost::geometry::model::polygon<boost::tuple<double, double>> boost_polygon;
	boost::geometry::read_wkt(wkt_string, boost_polygon);
	return boost_polygon;
}


Polygon convert_boost_polygon_to_polygon(const boost::geometry::model::polygon<boost::tuple<double, double>> boost_polygon){
	Polygon polygon;
	// For each boost polygon's vertex
	for (int i = 0; i < boost_polygon.outer().size() - 1; i++){
		Point point;
		// Get boost polygon's vertex coordinates
		point.x = boost_polygon.outer()[i].get<0>();
		point.y = boost_polygon.outer()[i].get<1>();
		polygon.push_back(point);
	}
	return polygon;
}


std::vector<Polygon> create_convex_hull(const std::vector<Polygon> obstacle_list){
	std::vector<Polygon> convex_hulls;
	for (int i = 0; i < obstacle_list.size(); i++){
		boost::geometry::model::polygon<boost::tuple<double, double>> boost_polygon = convert_polygon_to_boost_polygon(obstacle_list[i]);;
		boost::geometry::model::polygon<boost::tuple<double, double>> boost_convex_hull;
		// Apply the convex hull algorithm
		boost::geometry::convex_hull(boost_polygon, boost_convex_hull);
		Polygon polygon = convert_boost_polygon_to_polygon(boost_convex_hull);
		convex_hulls.push_back(polygon);
	}
	return convex_hulls;
}

