#include "offset.hpp"


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
		co.Execute(clipper_obstacle_with_offset, offset_value);

		// pass from clipper object to a polygon
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


// TODO: add the method to merge overlapping obstacles
// std::vector<Polygon> merge_obstacles(const std::vector<Polygon>& obstacle_list);


Polygon add_offset_to_borders(const Polygon& borders, float offset_value){
	Polygon borders_with_offset;
	ClipperLib::Path clipper_border;
	ClipperLib::Paths clipper_border_with_offset;

	// pass from borders to a clipper object
	for (const auto &position : borders) {
		clipper_border << ClipperLib::IntPoint(position.x * 1000.0, position.y * 1000.0);
	}

	// apply the offset
	ClipperLib::ClipperOffset co;
	co.AddPath(clipper_border, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
	co.Execute(clipper_border_with_offset, offset_value);

	// pass from clipper object to borders
	for(const ClipperLib::Path &path : clipper_border_with_offset){
		for(const ClipperLib::IntPoint &point: path){
			double x = point.X / 1000.0;
			double y = point.Y / 1000.0;
			borders_with_offset.emplace_back(x, y);
		}
	}
	return borders_with_offset;
}

