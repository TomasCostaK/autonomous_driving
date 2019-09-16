/*
	THIS FILE IS A PART OF GTA V SCRIPT HOOK SDK
				http://dev-c.com
			(C) Alexander Blade 2015
*/

#ifndef SCRIPT_H
#define SCRIPT_H

#pragma once

#include "inc\natives.h"
#include "inc\types.h"
#include "inc\enums.h"
#include "inc\main.h"
#include <string>
#include <vector>

// lidar GTA V parent directory
extern std::string lidarParentDir;
// logging file
extern std::string lidarLogFilePath;
// configurations file
extern std::string lidarCfgFilePath;
// error distance csv file path
extern std::string lidarErrorDistFilePath;
// number of different parameters available for the configuration file
extern int numCfgParams;

struct ray {
	bool hit;
	Vector3 hitCoordinates;
	Vector3 surfaceNormal;
	std::string entityTypeName;
	int entityTypeId;
	int rayResult;
	int hitEntityHandle;
};

struct ProjectedPointData {
	int screenCoordX;
	int screenCoordY;
	// 0 (120º), 1 (240º), 2 (360º)
	int pictureId;

	// determines if the point is already configured or not, in orther to avoid duplicated points in the LiDAR_PointCloud_points.txt and LiDAR_PointCloud_error.txt files
	bool stateSet = false;
};

void lidar(	double horiFovMin,
			double horiFovMax,
			double vertFovMin,
			double vertFovMax,
			double horiStep,
			double vertStep,
			int range,
			std::string filePath,
			double error,
			int errorDist,
			std::ofstream& log);

ray angleOffsetRaycast(double angleOffsetX, double angleOffsetZ, int range);

ray raycast(Vector3 source, Vector3 direction, float maxDistance, int intersectFlags);

void notificationOnLeft(std::string notificationText);

std::ifstream& GotoLineInPositionsDBFile(std::ifstream& inputfile, unsigned int num);

int CheckNumberOfLinesInFile(std::string filename);

int GetEncoderClsid(WCHAR* format, CLSID* pClsid);

int SaveScreenshot(std::string filename, ULONG uQuality);

void introduceError(Vector3* xyzError, 
					double x, 
					double y, 
					double z, 
					double error, 
					int errorType, 
					int range, 
					std::vector<double>& dist_vector, 
					std::vector<double>& error_vector);

double getError(std::vector<double>& dist, std::vector<double>& error, double r);

void readErrorFile(std::vector<double>& dist, std::vector<double>& error);

std::vector<double> split(const std::string& s, char delimiter);

//std::vector<std::string> loadConfigurationInfo(int numParams);

void ScriptMain();

#endif