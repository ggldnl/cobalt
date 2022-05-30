/**
 * @file utils.h
 * @author Re-L (danielgigliotti99.dg@gmail.com)
 * @brief 
 * 
 * utility class for frequently used functions
 * 
 * @version 0.1
 * @date 2022-03-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>

/**
 * @brief split the the string according to the provided character (default is space)
 * 
 * @param str 
 * @param c 
 * @return std::vector<std::string> 
 */
std::vector<std::string> split(const char *str, char c = ' ');

/**
 * @brief clamps the value v between the two provided thresholds
 * 
 */
template <typename T>
static T clamp(T v, T bottom, T top) {
	if (v > top)
		return top;
	if (v < bottom)
		return bottom;
	return v;
}

#endif