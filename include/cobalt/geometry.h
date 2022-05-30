
#ifndef GEOMETRY_H
#define GEOMETRY_H

/**
 * @brief Defines the geometry of the robot.
 * 
 * base_diameter = 170mm -> 0.17m
 * base_height = 70mm -> 0.07m
 * wheel_radius = 35mm -> 0.035m
 * wheel_distance = 80mm -> 0.08m
 *
 */
struct Geometry {
	float base_diameter = 0.17; // m
	float base_height = 0.07; // m
	float wheel_radius = 0.035; // m
	float wheel_distance = 0.08; // m
};
extern struct Geometry robot_geometry;

#endif