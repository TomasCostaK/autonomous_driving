#define _USE_MATH_DEFINES

#include "script.h"
#include "keyboard.h"
#include <ctime>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <windows.h>
#include <gdiplus.h>
#include <iterator>
#include <random>
#include "direct.h"
#include <stdlib.h>
#include <chrono>
#include <map>
#include <list>
#include <filesystem>
#include <regex>
#include "debug.h"

#pragma comment(lib, "gdiplus.lib")

#pragma warning(disable : 4244 4305) // double <-> float conversions

#define StopSpeed 0.00000000001f
#define NormalSpeed 1.0f

#define PI 3.14159265

static std::default_random_engine generator;
static std::uniform_real_distribution<double> distribution(-1.0, 1.0);

using namespace Gdiplus;
namespace fs = std::filesystem;

#pragma region Definition of data output files

/**
	File path global variables: include files for logging, lidar configurations, noise generation,
	point cloud labeling, stored positions and rotations of vehicle route landmarks.
	Convention:
		- names beginning with '_' belong to files inside a particular sample directory
		- names beginning with '/' belong to files inside the parent directory "LiDAR GTA V"
**/

// Output directory inside the game installation directory
std::string lidarParentDir = "LiDAR GTA V";

// log files
std::string lidarLogFilePath = lidarParentDir + "/log.txt";
std::string lidarFrameLogFilePathDebug = "_logSampleDebug.txt";

// configuration file
std::string lidarCfgFilePath = lidarParentDir + "/LiDAR GTA V.cfg";

// file for jitter generation (add noise to generated point cloud)
std::string lidarErrorDistFilePath = lidarParentDir + "/dist_error.csv";

// file with points and associated pointwise labels
std::string lidarPointLabelsFilePath = lidarParentDir + "/pointcloudLabels.txt";

// store route information character position and rotation
std::string routeFilePath = lidarParentDir + "/_positionsDB.txt";
std::string playerRotationsFilename = lidarParentDir + "/_rotationsDB.txt";

// lidar (and camera) distance to ground
std::string lidarHeightFilename = "_lidarHeight.txt";

// file holding vehicle information: position, rotation, projection coordinates, ...
std::string vehiclesInSampleFilename = "_vehicles_dims.txt";	

// character rotation
std::string playerRotationInSampleFilename = "_rotation.txt";

#pragma endregion

#pragma region Output streams

// Writing pointcloud segmentation using 4 classes (background, vehicle, human and animals, game props)
std::ofstream labelsFileStreamW;
// Writing segmentation by (gameobject) entity id (detailed)
std::ofstream labelsDetailedFileStreamW;
// Writing route positions
std::ofstream positionsDBFileW;
// Writing route rotation associated with the correspondent positions
std::ofstream playerRotationsStreamW;
// Writing output point cloud to a .ply file
std::ofstream fileOutput;
// Writing pointcloud points to a text file
std::ofstream fileOutputPoints;
// Writing output point cloud with noise to a .ply file
std::ofstream fileOutputError;
// Writing pointcloud points with noise to a text file
std::ofstream fileOutputErrorPoints;

#pragma endregion

#pragma region State variables

// Used to enable a sequence of lidar scans
bool takeSnap = false;
// Set when the player character has teleported to a given location. Makes it possible to impose a waiting time for the game assets to load.
bool playerTeleported = false;
// Set when position recording is taking place (F3 key).
bool isRecordingPositions = false;
// Set when the automatic LiDAR scanning process is in progress (initiated by F5 key).
bool isAutoScanning = false;
// Set when the character waited a certain time after teleporting (this is used to waiting for the game assets to load)
bool isWaitingAfterTeleportFinished = false;
// Set when a lidar scan is being executed
bool scanLive = false;
// Set at the beginning of a lidar scan for setting the environment
bool lidarScanPrep = false;
// Set when the player starts recording positions for the first time in the current instance of the game.
bool haveRecordedPositions = false;
// If the user has already performed (and interrupted) the automatic lidar scanning in the current instance of the game.
bool hasStartedAndStoppedAutoScan = false;			

#pragma endregion

#pragma region Counters

// Number of currently recorded positions (initiated by pressing the F3 key).
int positionsCounter = 0;
// Number of single scans performed (F2 key)
int singleScanCounter = 0;
// Number of lidar scans completed
int lidarScansCounter = 0;	
// number of lines of the file with the route positions
int positionsFileNumberOfLines = -1;

#pragma endregion

#pragma region Lidar and camera height

// It's used because the function GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS() gives the player's 
// position (at the middle of the character model, and not at foot level); in meters.
float halfCharacterHeight = 1.51;		// shouldn't be altered
// This variable is used to adjust the height at which to position the origin of the lidar/camera coordinate system. 
float raycastHeightParam = 2.2;

float lidarAndCamHeight = 1.73;

#pragma endregion

#pragma region Time intervals used during automatic scanning

// seconds between position recording
float secondsBetweenRecordings = 2;
int secondsToWaitAfterTeleport = 5;
// in order for the beginning of the next scan to not be sudden
int secondsBeforeStartingLidarScan = 2;				

#pragma endregion

#pragma region Point cloud generation and point projection

// camera/lidar position in world space
Vector3 centerDot;
// resolution of the computer screen
int resolutionX, resolutionY;
// reference to camera that takes the photos
Cam panoramicCam;
// projected coordinates of the ideal and noisy point clouds points. Values between 0 and 1 if they are inside the view frustum of the camera
float x2d, y2d, err_x2d, err_y2d;
// used to apply noise to the points
std::vector<double> dist_vector, error_vector;

int nHorizontalSteps;
int nVerticalSteps;
int no_of_cols;
int no_of_rows;

// aid in point cloud generation by splitting it's generation through multiple frames of the game
int nWorkloadFrames = 4; // used to configure the naumber of frames needed to generate a point cloud sample. Avoids the program window to turn unresponsive do to heavy calculations in a single frame
int countFrames;
int indexRowCounter = 0;
// xValue stores the x value to be calculated in the next frame
double xValue;

std::vector<std::vector<Vector3>> pointsMatrix;
std::vector<std::vector<Vector3>> pointsWithErrorMatrix;
std::vector<std::vector<ProjectedPointData>> pointsProjectedMatrix;
std::vector<std::vector<ProjectedPointData>> pointsProjectedWithErrorMatrix;

std::vector<std::vector<int>> labels;
std::vector<std::vector<int>> labelsDetailed;

std::vector<int> pointsPerVerticalStep;

// store information of different vehicles detected by the lidar
// Values per vehicle: "Entity | Hash | Vector3 minDimCoords | Projected minDimCoords | Vector3 maxDimCoords | Projected minDimCoords | Posx | Posy | Posz | Rotx | Roty | Rotz | Vehicle center Projection Coords"
std::map<Entity, std::string> vehiclesLookupTable;		

#pragma endregion

#pragma region Initial mod configurations


float vehicleDensityMultiplierLimit = 2;
float currentVehicleDenstiyMultiplier = 0;
float pedDensityMultiplierLimit = 2;
float currentPedDensityMultiplier = 0;

bool configsSet = false;
bool isInvincible = true;
float vehicleSpawnDistance = 30; // meters


#pragma endregion

void SetInitialConfigs()
{
	PLAYER::SET_PLAYER_INVINCIBLE(PLAYER::PLAYER_ID(), isInvincible);			// make player invincible

	//GAMEPLAY::SET_DISPATCH_IDEAL_SPAWN_DISTANCE(vehicleSpawnDistance);

	//Set clear weather
	GAMEPLAY::CLEAR_OVERRIDE_WEATHER();
	GAMEPLAY::SET_OVERRIDE_WEATHER("OVERCAST");

	//Set time to noon
	TIME::SET_CLOCK_TIME(12, 0, 0);
}

void SetPedAndVehicleDensityMultipliers()
{
	VEHICLE::SET_VEHICLE_DENSITY_MULTIPLIER_THIS_FRAME(currentVehicleDenstiyMultiplier);
	PED::SET_PED_DENSITY_MULTIPLIER_THIS_FRAME(currentPedDensityMultiplier);
}

void ScriptMain()
{
	srand(GetTickCount());

	std::ifstream positionsDBFileR;
	std::ifstream playerRotationsStreamR;

	auto start_time_for_collecting_positions = std::chrono::high_resolution_clock::now();
	auto start_time_after_teleport = std::chrono::high_resolution_clock::now();

	// event handling loop
	while (true)
	{
		if (!configsSet)
		{
			SetInitialConfigs();
			configsSet = true;
		}
		
		// required per frame
		SetPedAndVehicleDensityMultipliers();

		// keyboard commands information
		if (IsKeyJustUp(VK_F1))
		{
			notificationOnLeft("- F2: Scan current environment\n- F3: start/stop recording player position\n- F4: scripthook V menu\n- F5: start/stop automatic scanning");
		}

		// decrease vehicle and ped density multipliers
		if (IsKeyJustUp(VK_F6))
		{
			if (currentPedDensityMultiplier >= 0.1)
				currentPedDensityMultiplier -= 0.1;
			
			if (currentVehicleDenstiyMultiplier >= 0.1)
				currentVehicleDenstiyMultiplier -= 0.1;

			notificationOnLeft("Ped density set to: " + std::to_string(currentPedDensityMultiplier) + "\nVehicle density set to: " + std::to_string(currentVehicleDenstiyMultiplier));
		}

		// increase vehicle and ped density multipliers
		if (IsKeyJustUp(VK_F7))
		{
			if (currentPedDensityMultiplier < pedDensityMultiplierLimit)
				currentPedDensityMultiplier += 0.1;

			if (currentVehicleDenstiyMultiplier < vehicleDensityMultiplierLimit)
				currentVehicleDenstiyMultiplier += 0.1;

			notificationOnLeft("Ped density set to: " + std::to_string(currentPedDensityMultiplier) + "\nVehicle density set to: " + std::to_string(currentVehicleDenstiyMultiplier));
		}

		// teleport the character to the start position of the route, or the landmark from where the automatic scanning was interrupted/stopped
		if (IsKeyJustUp(VK_F2) && !isAutoScanning && !isRecordingPositions)
		{
			scanLive = true;
			takeSnap = true;
			singleScanCounter++;
		}

		// start or stop recording the player's positions
		if (IsKeyJustUp(VK_F3) && !isAutoScanning)
		{
			try
			{
				if (!isRecordingPositions)
				{
					start_time_for_collecting_positions = std::chrono::high_resolution_clock::now();

					if (haveRecordedPositions)
					{
						// doesnt overwrite the file
						positionsDBFileW.open(routeFilePath, std::ios_base::app);	// open file in append mode
						playerRotationsStreamW.open(playerRotationsFilename, std::ios_base::app);
						notificationOnLeft("Player position recording has restarted!");
					}
					else
					{
						// overwrites the file
						positionsDBFileW.open(routeFilePath);						// open file in overwrite mode
						playerRotationsStreamW.open(playerRotationsFilename);
						notificationOnLeft("Player position recording has started!");
					}

					isRecordingPositions = true;
				}
				else
				{
					positionsDBFileW.close();	// close file
					playerRotationsStreamW.close();
					isRecordingPositions = false;
					notificationOnLeft("Player position recording has finished!");
				}
			}
			catch (std::exception &e)
			{
				notificationOnLeft(e.what());
				return;
			}
		}

		// get current player position and store it in a file
		if (isRecordingPositions)
		{
			auto current_time = std::chrono::high_resolution_clock::now();

			double elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_for_collecting_positions).count();

			if (elapsedTime > secondsBetweenRecordings)
			{
				positionsCounter++;
				// get player position (middle of the character model)
				Vector3 playerCurrentPos = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(PLAYER::PLAYER_PED_ID(), 0, 0, 0);/*-halfCharacterHeight);*/

				// write current player position to the text file
				positionsDBFileW << std::to_string(playerCurrentPos.x) + " " + std::to_string(playerCurrentPos.y) + " " + std::to_string(playerCurrentPos.z) + "\n";

				float rotx, roty, rotz;
				
				Vector3 rot = ENTITY::GET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), 0);

				rotx = rot.x;
				roty = rot.y;
				rotz = rot.z;

				playerRotationsStreamW << std::to_string(rotx) + " " + std::to_string(roty) + " " + std::to_string(rotz) + "\n";

				haveRecordedPositions = true;

				// reset start time
				start_time_for_collecting_positions = std::chrono::high_resolution_clock::now();

				notificationOnLeft("Number of positions: " + std::to_string(positionsCounter));
			}
		}

		// start or stop gathering lidar data
		if (IsKeyJustUp(VK_F5) && !isRecordingPositions)
		{
			if (CheckNumberOfLinesInFile(routeFilePath) == 0) // the positions file is empty
			{
				notificationOnLeft("There aren't any positions stored in the file.\n\nPress F3 to record new positions!");
				continue;
			}

			if (CheckNumberOfLinesInFile(routeFilePath) - (lidarScansCounter - singleScanCounter) > 0) // if not all the positions in the file were scanned, continue with the scanning of the rest of the positions
			{
				try
				{
					if (!isAutoScanning)
					{
						positionsFileNumberOfLines = CheckNumberOfLinesInFile(routeFilePath);

						positionsDBFileR.open(routeFilePath);	// open file
						playerRotationsStreamR.open(playerRotationsFilename);
						isAutoScanning = true;

						if (!hasStartedAndStoppedAutoScan)
						{
							notificationOnLeft("Lidar scanning starting in " + std::to_string((int)(secondsToWaitAfterTeleport)) + " seconds" + "\n\nTotal number of positions: " + std::to_string(positionsFileNumberOfLines));
							hasStartedAndStoppedAutoScan = true; // TODO: complete thought process
						}
						else
						{
							notificationOnLeft("Lidar scanning restarting in " + std::to_string((int)(secondsToWaitAfterTeleport)) + " seconds" + "\n\nRemaining positions: " + std::to_string(positionsFileNumberOfLines - lidarScansCounter));
							GotoLineInPositionsDBFile(positionsDBFileR, (lidarScansCounter - singleScanCounter) + 1);
						}
					}
					else
					{
						playerRotationsStreamR.close();
						positionsDBFileR.close();	// close file
						isAutoScanning = false;
						notificationOnLeft("Lidar scanning was stopped before the end of the file!\n\nTo continue from the latest position, press F5 when ready!");
					}
				}
				catch (std::exception &e)
				{
					notificationOnLeft(e.what());
					return;
				}
			}
			else
			{
				notificationOnLeft("All positions were scanned!\n\nTo continue the LiDAR scanning add more positions to the file with F3.");
				continue;
			}
		}

		// gathering point cloud and image data according to the positions read from a file
		if (isAutoScanning && !takeSnap && !scanLive)
		{
			float xPos, yPos, zPos;
			float xRot, yRot, zRot;
			// get next position to teleport to, and teleport the character
			if (positionsDBFileR >> xPos >> yPos >> zPos && playerRotationsStreamR >> xRot >> yRot >> zRot)
			{
				Vector3 playerPos;
				// update global variable
				playerPos.x = xPos;
				playerPos.y = yPos;
				playerPos.z = zPos;

				PED::SET_PED_COORDS_NO_GANG(PLAYER::PLAYER_PED_ID(), playerPos.x, playerPos.y, playerPos.z);
				// reorient the player in order for the pictures to start to be taken from the view of the player
				ENTITY::SET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), xRot, yRot, zRot, 0, 0);

				// in order to prevent the character from automatically rotating to the last input direction, repeat the teleport and reorientation
				PED::SET_PED_COORDS_NO_GANG(PLAYER::PLAYER_PED_ID(), playerPos.x, playerPos.y, playerPos.z);
				ENTITY::SET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), xRot, yRot, zRot, 0, 0);

				playerTeleported = true;
				isWaitingAfterTeleportFinished = false;
			}
			else // reached the end of the file
			{
				isAutoScanning = false;
				positionsDBFileR.close();	// close file
				playerRotationsStreamR.close();
				notificationOnLeft("Lidar scanning completed!");
			}
		}

		// if player was teleported, wait a few seconds before the LiDAR scanning starts
		if (isAutoScanning && playerTeleported)
		{
			scanLive = true;

			playerTeleported = false;

			start_time_after_teleport = std::chrono::high_resolution_clock::now();
		}

		if (isAutoScanning && !playerTeleported && !isWaitingAfterTeleportFinished)
		{
			auto currentTime = std::chrono::high_resolution_clock::now();

			double elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - start_time_after_teleport).count();

			if (elapsedTime > secondsToWaitAfterTeleport)
			{
				takeSnap = true;
				isWaitingAfterTeleportFinished = true;
			}
		}

		// output stream for writing log information into the log.txt file
		std::ofstream log;
		
		if (takeSnap)
		{
			// reset
			takeSnap = false;
			if (isAutoScanning) // in order to not increase the counter when doing manual snapshots
				lidarScansCounter++;

			log.open(lidarLogFilePath, std::ios_base::app);

			try
			{
				double parameters[6];
				int rangeRay;
				int errorDist;
				double error;
				std::string filename;		// name for the point cloud file (.ply) and it's parent directory
				std::string ignore;
				std::ifstream inputFile;

				// load lidar configurations
				inputFile.open(lidarCfgFilePath);
				if (inputFile.bad()) {
					notificationOnLeft("Input file not found. Please re-install the plugin.");
					return;
				}
				log << "Reading configuration file...\n";
				inputFile >> ignore >> ignore >> ignore >> ignore >> ignore;
				for (int i = 0; i < 6; i++) {
					inputFile >> ignore >> ignore >> parameters[i];
				}
				inputFile >> ignore >> ignore >> rangeRay;
				inputFile >> ignore >> ignore >> filename;
				inputFile >> ignore >> ignore >> error;
				inputFile >> ignore >> ignore >> errorDist;

				inputFile.close();

				// start lidar process
				std::string newfolder = "LiDAR GTA V/" + filename;
				log << "Name: " + newfolder + "\n";

				lidarScansCounter = getNumberOfOutputDir("LiDAR GTA V/", filename, log);
				newfolder += std::to_string(lidarScansCounter);

				_mkdir(newfolder.c_str());

				// prepare for lidar scan
				SetupGameForLidarScan(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], newfolder + "/" + filename, log);

				log << "Setup done\n";

				log.close();
			}
			catch (std::exception &e)
			{
				notificationOnLeft(e.what());
				log << e.what();
				log.close();
				return;
			}
		}

		if (scanLive && lidarScanPrep)
		{
			double parameters[6];
			int rangeRay;
			int errorDist;
			double error;
			std::string filename;		// name for the point cloud file (.ply) and it's parent directory
			std::string ignore;
			std::ifstream inputFile;

			// load lidar configurations
			inputFile.open(lidarCfgFilePath);
			if (inputFile.bad()) {
				notificationOnLeft("Input file not found. Please re-install the plugin.");
				return;
			}
			//log << "Reading input file...\n";
			inputFile >> ignore >> ignore >> ignore >> ignore >> ignore;
			for (int i = 0; i < 6; i++) {
				inputFile >> ignore >> ignore >> parameters[i];
			}
			inputFile >> ignore >> ignore >> rangeRay;
			inputFile >> ignore >> ignore >> filename;
			inputFile >> ignore >> ignore >> error;
			inputFile >> ignore >> ignore >> errorDist;

			inputFile.close();

			// start lidar process
			std::string newfolder = "LiDAR GTA V/" + filename + std::to_string(lidarScansCounter);

			lidar(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], rangeRay, newfolder + "/" + filename, error, errorDist, log);

			if (countFrames == nWorkloadFrames)
			{
				scanLive = false;
				lidarScanPrep = false;
				PostLidarScanProcessing(newfolder + "/" + filename);

				if (isAutoScanning)
				{
					int percentageComplete = ((float)lidarScansCounter - singleScanCounter) / ((float)positionsFileNumberOfLines) * 100;

					if (percentageComplete > 0)	// ignore when the scan is done by clicking f2
						notificationOnLeft("Snapshots taken: " + std::to_string(lidarScansCounter - singleScanCounter) + "\n\nCompleted: " + std::to_string(percentageComplete) + "%");
				}
			}
		}

		WAIT(0);
	}
}

int getNumberOfOutputDir(std::string parentDir, std::string parentDirname, std::ofstream& log)
{
	int dirMaxNum = 0;		// biggest "LiDAR_PointCloudN" number 
	std::string path = parentDir;
	for (const auto &entry : fs::directory_iterator(path))
	{
		log << "iter dir: " + entry.path().string() + "\n";
		if (entry.path().string().find(parentDirname) != std::string::npos) {
			//std::cout << "found!" << '\n';
			std::string dirname = entry.path().string();
			std::string dirNumberStr = std::regex_replace(dirname, std::regex("([^0-9])"), "");
			log << "\t\tNumber: " + dirNumberStr + "\n";
			int dirNum = std::stoi(dirNumberStr);
			if (dirNum > dirMaxNum)
				dirMaxNum = dirNum;
		}

		//std::cout << entry.path() << std::endl;
	}
	dirMaxNum += 1;
	return dirMaxNum;
}

int CheckNumberOfLinesInFile(std::string filename)
{
	// https://stackoverflow.com/questions/3072795/how-to-count-lines-of-a-file-in-c
	std::ifstream inFile(filename);
	return std::count(std::istreambuf_iterator<char>(inFile), std::istreambuf_iterator<char>(), '\n');
}

std::ifstream& GotoLineInPositionsDBFile(std::ifstream& inputfile, unsigned int num)
{
	if (num == 0)
	{
		return inputfile;
	}

	std::string ignore;
	for (int i = 0; i < num - 1; ++i) {
		inputfile >> ignore >> ignore >> ignore;	// each line of the positions file has 3 numbers separated by spaces
	}

	return inputfile;
}

std::vector<double> split(const std::string& s, char delimiter)
{
	std::vector<double> tokens;
	std::string token;
	std::istringstream tokenStream(s);
	while (std::getline(tokenStream, token, delimiter))
	{
		tokens.push_back(std::stod(token));
	}
	return tokens;
}

void readErrorFile(std::vector<double>& dist, std::vector<double>& error) {
	std::ifstream inputFile;
	inputFile.open(lidarErrorDistFilePath);
	std::string errorDists, dists;
	inputFile >> errorDists;
	inputFile >> dists;

	dist = split(dists, ',');
	error = split(errorDists, ',');
}

double getError(std::vector<double>& dist, std::vector<double>& error, double r) {
	int x = 0;
	if (r == 0)
		return 0.0;
	int tmp = abs(r);
	while (dist[x] <= tmp) {
		if (dist[x] == tmp)
			return error[x];
		x++;
	}
	double fact;
	if (x == 0) {
		return 0;
	}
	fact = (abs(r) - dist[x - 1]) / (dist[x] - dist[x - 1]);
	return (1 - fact) * error[x - 1] + (fact)* error[x];
}

void introduceError(Vector3* xyzError, double x, double y, double z, double error, int errorType, int range, std::vector<double>& dist_vector, std::vector<double>& error_vector)
{
	//Convert to spherical coordinates
	double hxy = std::hypot(x, y);
	double r = std::hypot(hxy, z);
	double el = std::atan2(z, hxy);
	double az = std::atan2(y, x);

	// the error type is defined in the configurations file
	if (errorType == 0)
		r = r + distribution(generator) * error;
	else if (errorType == 1)
		r = r + distribution(generator) * error * r;
	else {
		if (r != 0)
			r = r + distribution(generator) * getError(dist_vector, error_vector, r);
	}

	//Convert to cartesian coordinates
	double rcos_theta = r * std::cos(el);
	xyzError->x = rcos_theta * std::cos(az);
	xyzError->y = rcos_theta * std::sin(az);
	xyzError->z = r * std::sin(el);
}

void notificationOnLeft(std::string notificationText) {
	UI::_SET_NOTIFICATION_TEXT_ENTRY("CELL_EMAIL_BCON");
	const int maxLen = 99;
	for (int i = 0; i < notificationText.length(); i += maxLen) {
		std::string divideText = notificationText.substr(i, min(maxLen, notificationText.length() - i));
		const char* divideTextAsConstCharArray = divideText.c_str();
		char* divideTextAsCharArray = new char[divideText.length() + 1];
		strcpy_s(divideTextAsCharArray, divideText.length() + 1, divideTextAsConstCharArray);
		UI::_ADD_TEXT_COMPONENT_STRING(divideTextAsCharArray);
	}
	int handle = UI::_DRAW_NOTIFICATION(false, 1);
}

ray raycast(Vector3 source, Vector3 direction, float maxDistance, int intersectFlags) {
	ray result;
	float targetX = source.x + (direction.x * maxDistance);
	float targetY = source.y + (direction.y * maxDistance);
	float targetZ = source.z + (direction.z * maxDistance);

	int rayHandle = WORLDPROBE::_CAST_RAY_POINT_TO_POINT(source.x, source.y, source.z, targetX, targetY, targetZ, intersectFlags, PLAYER::PLAYER_PED_ID(), 7);
	int hit = 0;
	int hitEntityHandle = -1;

	Vector3 hitCoordinates;
	hitCoordinates.x = 0;
	hitCoordinates.y = 0;
	hitCoordinates.z = 0;

	Vector3 surfaceNormal;
	surfaceNormal.x = 0;
	surfaceNormal.y = 0;
	surfaceNormal.z = 0;

	int rayResult = WORLDPROBE::_GET_RAYCAST_RESULT(rayHandle, &hit, &hitCoordinates, &surfaceNormal, &hitEntityHandle);
	result.rayResult = rayResult;
	result.hit = hit;
	result.hitCoordinates = hitCoordinates;
	result.surfaceNormal = surfaceNormal;
	result.hitEntityHandle = hitEntityHandle;	// true id of the object that the raycast collided with

	std::string entityTypeName = "RoadsBuildings";	// default name for hitEntityHandle = -1
	result.entityTypeId = -1;						// the entityTypeId field is a general id, that groups all the gameobjects into just 4 categories
	if (ENTITY::DOES_ENTITY_EXIST(hitEntityHandle)) // if the raycast intercepted an object
	{
		int entityType = ENTITY::GET_ENTITY_TYPE(hitEntityHandle);

		if (entityType == 1) {
			entityTypeName = "HumansAnimals";
		}
		else if (entityType == 2) {
			entityTypeName = "Vehicle";
		}
		else if (entityType == 3) {
			entityTypeName = "GameProp";
		}
		result.entityTypeId = entityType;
	}

	result.entityTypeName = entityTypeName;
	return result;
}

ray angleOffsetRaycast(double angleOffsetX, double angleOffsetZ, int range)
{
	Vector3 rot = CAM::GET_GAMEPLAY_CAM_ROT(2);
	double rotationX = (angleOffsetX) * (M_PI / 180.0);
	double rotationZ = (angleOffsetZ) * (M_PI / 180.0);
	double multiplyXY = abs(cos(rotationX));
	Vector3 direction;
	direction.x = sin(rotationZ) * multiplyXY * -1;
	direction.y = cos(rotationZ) * multiplyXY;
	direction.z = sin(rotationX);

	Vector3 raycastCenterPos = centerDot;

	ray result = raycast(raycastCenterPos, direction, range, -1);

	return result;
}

int GetEncoderClsid(WCHAR* format, CLSID* pClsid)
{
	unsigned int num = 0, size = 0;
	GetImageEncodersSize(&num, &size);
	if (size == 0) return -1;
	ImageCodecInfo* pImageCodecInfo = (ImageCodecInfo*)(malloc(size));
	if (pImageCodecInfo == NULL) return -1;
	GetImageEncoders(num, size, pImageCodecInfo);

	for (unsigned int j = 0; j < num; ++j) {
		if (wcscmp(pImageCodecInfo[j].MimeType, format) == 0) {
			*pClsid = pImageCodecInfo[j].Clsid;
			free(pImageCodecInfo);
			return j;
		}
	}
	free(pImageCodecInfo);
	return -1;
}

int SaveScreenshot(std::string filename, ULONG uQuality = 100)
{
	ULONG_PTR gdiplusToken;
	GdiplusStartupInput gdiplusStartupInput;
	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
	HWND hMyWnd = GetDesktopWindow();
	RECT r;
	int w, h;
	HDC dc, hdcCapture;
	int nBPP, nCapture, iRes;
	LPBYTE lpCapture;
	CLSID imageCLSID;
	Bitmap* pScreenShot;

	// get the area of my application's window     
	GetWindowRect(hMyWnd, &r);
	dc = GetWindowDC(hMyWnd);   // GetDC(hMyWnd) ;
	w = r.right - r.left;
	h = r.bottom - r.top;
	nBPP = GetDeviceCaps(dc, BITSPIXEL);
	hdcCapture = CreateCompatibleDC(dc);

	// create the buffer for the screenshot
	BITMAPINFO bmiCapture = { sizeof(BITMAPINFOHEADER), w, -h, 1, nBPP, BI_RGB, 0, 0, 0, 0, 0, };

	// create a container and take the screenshot
	HBITMAP hbmCapture = CreateDIBSection(dc, &bmiCapture, DIB_PAL_COLORS, (LPVOID*)& lpCapture, NULL, 0);

	// failed to take it
	if (!hbmCapture) {
		DeleteDC(hdcCapture);
		DeleteDC(dc);
		GdiplusShutdown(gdiplusToken);
		printf("failed to take the screenshot. err: %d\n", GetLastError());
		return 0;
	}

	// copy the screenshot buffer
	nCapture = SaveDC(hdcCapture);
	SelectObject(hdcCapture, hbmCapture);
	BitBlt(hdcCapture, 0, 0, w, h, dc, 0, 0, SRCCOPY);
	RestoreDC(hdcCapture, nCapture);
	DeleteDC(hdcCapture);
	DeleteDC(dc);

	// save the buffer to a file   
	pScreenShot = new Bitmap(hbmCapture, (HPALETTE)NULL);
	EncoderParameters encoderParams;
	encoderParams.Count = 1;
	encoderParams.Parameter[0].NumberOfValues = 1;
	encoderParams.Parameter[0].Guid = EncoderQuality;
	encoderParams.Parameter[0].Type = EncoderParameterValueTypeLong;
	encoderParams.Parameter[0].Value = &uQuality;
	GetEncoderClsid(L"image/jpeg", &imageCLSID);

	wchar_t* lpszFilename = new wchar_t[filename.length() + 1];
	mbstowcs(lpszFilename, filename.c_str(), filename.length() + 1);

	iRes = (pScreenShot->Save(lpszFilename, &imageCLSID, &encoderParams) == Ok);
	delete pScreenShot;
	DeleteObject(hbmCapture);
	GdiplusShutdown(gdiplusToken);
	return iRes;
}

void SetLidarHeight()
{
	Vector3 origin;
	origin.x = centerDot.x;
	origin.y = centerDot.y;
	origin.z = centerDot.z;

	Vector3 down;
	down.x = 0;
	down.y = 0;
	down.z = -1;

	ray lidarToGround = raycast(origin, down, 2.5, -1);
	Vector3 collisionPoint = lidarToGround.hitCoordinates;
	centerDot.z = collisionPoint.z + lidarAndCamHeight;
}


// this function is only called once at the beginning of each lidar scan
void SetupGameForLidarScan(double horiFovMin, double horiFovMax, double vertFovMin, double vertFovMax, double horiStep, double vertStep, std::string filePath, std::ofstream& log)
{
	readErrorFile(dist_vector, error_vector);

	//centerDot = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(PLAYER::PLAYER_PED_ID(), 0, 0, raycastHeightParam - halfCharacterHeight);
	centerDot = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(PLAYER::PLAYER_PED_ID(), 0, 0, 0);
	SetLidarHeight();

	// write to file the lidar height 
	LogLidarHeight(filePath + lidarFrameLogFilePathDebug, "SetupGameForLidarScan", centerDot);

	LogPlayerAlphaValue(filePath + lidarFrameLogFilePathDebug, "SetupGameForLidarScan");
	
	ENTITY::SET_ENTITY_ALPHA(PLAYER::PLAYER_PED_ID(), 0, true);			// make player invisible; alphaLevel between 0 and 255

	GRAPHICS::GET_SCREEN_RESOLUTION(&resolutionX, &resolutionY);

	LogPlayerWorldPos(filePath + lidarFrameLogFilePathDebug, "SetupGameForLidarScan");
	LogPlayerWorldRot(filePath + lidarFrameLogFilePathDebug, "SetupGameForLidarScan");

	//log << "Setting up camera...";
	Vector3 playerCurRot = ENTITY::GET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), 0);

	panoramicCam = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", 1);
	LogRenderingCameraSettings(filePath + lidarFrameLogFilePathDebug, "SetupGameForLidarScan", panoramicCam);
	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "SetupGameForLidarScan", "New Cam height pos", std::to_string(centerDot.x) + " " + std::to_string(centerDot.y) + " " + std::to_string(centerDot.z));

	CAM::SET_CAM_FOV(panoramicCam, 50);
	CAM::SET_CAM_ROT(panoramicCam, playerCurRot.x, playerCurRot.y, playerCurRot.z, 1);

	// put camera and lidar at the center of the car, but at the desired height
	if (PED::IS_PED_IN_ANY_VEHICLE(PLAYER::PLAYER_PED_ID(), true))
	{
		Vector3 rightVec;
		rightVec.x = 1; 
		rightVec.y = 0;
		rightVec.z = 0;
		rightVec = rotate_point_around_z_axis(rightVec, playerCurRot.z);

		Vehicle pedVehicle = PED::GET_VEHICLE_PED_IS_IN(PLAYER::PLAYER_PED_ID(), false);

		// stop the character vehicle from moving
		ENTITY::SET_ENTITY_VELOCITY(pedVehicle, 0, 0, 0);

		Hash vehicleHash = ENTITY::GET_ENTITY_MODEL(pedVehicle);

		Vector3 vehiclePos = ENTITY::GET_ENTITY_COORDS(pedVehicle, 0);
		/*
		// get gameobject dimensions
		Vector3 minCoords;	// min XYZ
		Vector3 maxCoords;  // max XYZ

		GAMEPLAY::GET_MODEL_DIMENSIONS(vehicleHash, &minCoords, &maxCoords);*/
		centerDot.x = vehiclePos.x;
		centerDot.y = vehiclePos.y;
	}

	//CAM::SET_CAM_COORD(panoramicCam, centerDot.x, centerDot.y, centerDot.z);
	
	CAM::ATTACH_CAM_TO_ENTITY(panoramicCam, PLAYER::PLAYER_PED_ID(), 0, 6, raycastHeightParam - halfCharacterHeight, 1);
	//CAM::ATTACH_CAM_TO_ENTITY(panoramicCam, PLAYER::PLAYER_PED_ID(), 0, 0, GetLidarHeight(), 1);
	
	CAM::RENDER_SCRIPT_CAMS(1, 0, 0, 1, 0);
	CAM::SET_CAM_ACTIVE(panoramicCam, true);
	WAIT(50);

	LogRenderingCameraSettings(filePath + lidarFrameLogFilePathDebug, "SetupGameForLidarScan", panoramicCam);

	LogMaxNumberOfPoints(filePath + lidarFrameLogFilePathDebug, "SetupGameForLidarScan", horiFovMin, horiFovMax, horiStep, vertFovMin, vertFovMax, vertStep);

	fileOutput.open(filePath + ".ply");
	fileOutputPoints.open(filePath + "_points.txt");
	fileOutputError.open(filePath + "_error.ply");
	fileOutputErrorPoints.open(filePath + "_error.txt");
	labelsFileStreamW.open(filePath + "_labels.txt");					// open labels file
	labelsDetailedFileStreamW.open(filePath + "_labelsDetailed.txt");	// open labels file
	 
	//Disable HUD and Radar
	UI::DISPLAY_HUD(false);
	UI::DISPLAY_RADAR(false);

	GAMEPLAY::SET_TIME_SCALE(StopSpeed);

	nHorizontalSteps = (horiFovMax - horiFovMin) / horiStep;
	nVerticalSteps = (vertFovMax - vertFovMin) / vertStep;

	no_of_cols = nHorizontalSteps;
	no_of_rows = nVerticalSteps;

	// It's a vector containing no_of_rows vectors containing no_of_cols points.
	pointsMatrix = std::vector<std::vector<Vector3>>(no_of_rows + 1, std::vector<Vector3>(no_of_cols + 1));
	pointsWithErrorMatrix = std::vector<std::vector<Vector3>>(no_of_rows + 1, std::vector<Vector3>(no_of_cols + 1));
	pointsProjectedMatrix = std::vector<std::vector<ProjectedPointData>>(no_of_rows + 1, std::vector<ProjectedPointData>(no_of_cols + 1));
	pointsProjectedWithErrorMatrix = std::vector<std::vector<ProjectedPointData>>(no_of_rows + 1, std::vector<ProjectedPointData>(no_of_cols + 1));

	labels = std::vector<std::vector<int>>(no_of_rows + 1, std::vector<int>(no_of_cols + 1));
	labelsDetailed = std::vector<std::vector<int>>(no_of_rows + 1, std::vector<int>(no_of_cols + 1));

	pointsPerVerticalStep = std::vector<int>(nVerticalSteps + 1);

	// initialize loop values to their initial ones
	xValue = vertFovMin;
	
	countFrames = 0;
	indexRowCounter = 0;

	lidarScanPrep = true;

	// remove all elements from the dictionary
	vehiclesLookupTable.clear();
}

float distanceBetween3dPoints(Vector3 p1, Vector3 p2)
{
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}

float dotProduct3D(Vector3 v1, Vector3 v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

// returns a vector with: minCornerPos, maxCornerPos
std::vector<Vector3> GetBestMinMaxCoords(std::vector<Vector3> corners, Vector3 vehiclePos, int& truncated, Vector3 canonicalMinCoords, Vector3 canonicalMaxCoords, std::string filePath)
{
	truncated = 0;	// if the vehicle is totally or partly inside the image

	float trueDiagonal = distanceBetween3dPoints(canonicalMinCoords, canonicalMaxCoords);
	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "True Diagonal length", std::to_string(trueDiagonal));

	Vector3 c1;
	c1.x = 0;
	c1.y = 0;
	c1.z = 0;
	Vector3 c2;
	c2.x = 0;
	c2.y = 0;
	c2.z = 0;
	std::vector<Vector3> finalMinMaxCorners = { c1 , c2 };		// apenas para inicialização

	float area = -1;
	int areaProjected = -1;

	std::string line = "";

	for (int i = 0; i < 8; i++)
	{
		Vector3 corner1 = corners[i];
		
		for (int j = 0; j < 8; j++)
		{
			// ignore the same corner as corner1
			if (j == i)
				continue;

			Vector3 corner2 = corners[j];

			float minxProjected;
			float minyProjected;
			// returns (-1, -1) when the projected point is outside the image view
			GRAPHICS::_WORLD3D_TO_SCREEN2D(corner1.x + vehiclePos.x, corner1.y + vehiclePos.y, corner1.z + vehiclePos.z, &minxProjected, &minyProjected);

			float maxxProjected;
			float maxyProjected;
			GRAPHICS::_WORLD3D_TO_SCREEN2D(corner2.x + vehiclePos.x, corner2.y + vehiclePos.y, corner2.z + vehiclePos.z, &maxxProjected, &maxyProjected);

			// before calculating the area, the coordinates need to be evaluated in order to form the min and max 2D points from the alctual min and max coordinates
			float tmp = 0;

			LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Iteration", "[" + std::to_string(i) + "][" + std::to_string(j) + "]");
			LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Min Projected Coords", "" + std::to_string(minxProjected) + ", " + std::to_string(minyProjected) + "");
			LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Max Projected Coords", "" + std::to_string(maxxProjected) + ", " + std::to_string(maxyProjected) + "");
			//LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Area before", std::to_string(abs((int)(minxProjected*1.5*resolutionX) - (int)(maxxProjected * 1.5*resolutionX)) * abs((int)(minyProjected*1.5*resolutionY) - (int)(maxyProjected * 1.5*resolutionY))));

			/*if (minxProjected > maxxProjected)
			{
				tmp = minxProjected;
				minxProjected = maxxProjected;
				maxxProjected = tmp;
			}

			if (minyProjected > maxyProjected)
			{
				tmp = minyProjected;
				minyProjected = maxyProjected;
				maxyProjected = tmp;
			}*/

			// area with projected points
			int tmpAreaProjected = abs((int)(minxProjected*1.5*resolutionX) - (int)(maxxProjected * 1.5*resolutionX)) * abs((int)(minyProjected*1.5*resolutionY) - (int)(maxyProjected * 1.5*resolutionY));

			
			LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Corner1", std::to_string(corner1.x) + " " + std::to_string(corner1.y) + " " + std::to_string(corner1.z));
			LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Corner2", std::to_string(corner2.x) + " " + std::to_string(corner2.y) + " " + std::to_string(corner2.z));

			LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Area", std::to_string(tmpAreaProjected));

			// determine if the area is bigger than the previously bigger area, and if the point is projected inside the image view
			if (tmpAreaProjected > areaProjected &&
				(minxProjected*1.5*resolutionX) > 0 && (minyProjected*1.5*resolutionY) > 0 &&
				(maxxProjected*1.5*resolutionX) > 0 && (maxyProjected*1.5*resolutionY) > 0)
			{
				// ensure that the two 3d corners dont lie on the same plane in any of the axis (using the box dimesions), because the croners that give the biggest 2d bounding box can lie on the same plane, which is not desirable
				// 1º) verify, and correct when not, if the min coords are less than the max coords
				float tmp2 = 0;
				if (corner1.x > corner2.x)
				{
					tmp2 = corner1.x;
					corner1.x = corner2.x;
					corner2.x = tmp2;
				}
				if (corner1.y > corner2.y)
				{
					tmp2 = corner1.y;
					corner1.y = corner2.y;
					corner2.y = tmp2;
				}
				if (corner1.z > corner2.z)
				{
					tmp2 = corner1.z;
					corner1.z = corner2.z;
					corner2.z = tmp2;
				}

				LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Corner1 Reordered", std::to_string(corner1.x) + " " + std::to_string(corner1.y) + " " + std::to_string(corner1.z));
				LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Corner2 Reordered", std::to_string(corner2.x) + " " + std::to_string(corner2.y) + " " + std::to_string(corner2.z));

				// 2º) verify if the corners lie on the same plane by calculating the diagonal and compare it to the diagonal of the 3d box surround the car
				float testDiagonal = distanceBetween3dPoints(corner1, corner2);

				LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "GetBestMinMaxCoords", "Test Diagonal", std::to_string(testDiagonal));

				if (testDiagonal == trueDiagonal)	// means that all coordinates of the corners lie on different planes
				{
					areaProjected = tmpAreaProjected;
					finalMinMaxCorners[0].x = corner1.x;
					finalMinMaxCorners[0].y = corner1.y;
					finalMinMaxCorners[0].z = corner1.z;
					finalMinMaxCorners[1].x = corner2.x;
					finalMinMaxCorners[1].y = corner2.y;
					finalMinMaxCorners[1].z = corner2.z;
				}
				else
				{
					// skip
				}
			}
			else
				truncated = 1;
		}
	}

	return finalMinMaxCorners;
}

// arithmetic average between the 8 points of the box surrounding the vehicle
Vector3 FindCenterCoordsOf3Dbox(std::vector<Vector3> corners, Vector3 vehiclePos)
{
	Vector3 center;
	center.x = 0;
	center.y = 0;
	center.z = 0;
	
	// find X average
	for (int i = 0; i < 8; i++)
	{
		center.x += corners[i].x + vehiclePos.x;
	}
	center.x = center.x / 8;

	// find Y average
	for (int i = 0; i < 8; i++)
	{
		center.y += corners[i].y + vehiclePos.y;
	}
	center.y = center.y / 8;

	// find Z average
	for (int i = 0; i < 8; i++)
	{
		center.z += corners[i].z + vehiclePos.z;
	}
	center.z = center.z / 8;

	return center;
}

void RegisterVehicleInformation(Entity vehicleHandle, std::string entityType, std::string filePath)
{
	if (entityType.compare("Vehicle") != 0)		// the entity is not a vehicle
		return;

	// check if vehicle is already in the dictionary, if so, ignore the point
	if (vehiclesLookupTable.count(vehicleHandle) == 1) // if there is already 
		return;

	LogVehicleRotations(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", vehicleHandle);

	// stop the vehicle from moving
	ENTITY::SET_ENTITY_VELOCITY(vehicleHandle, 0, 0, 0);

	// get hash value of the corresponding vehicle model
	Hash vehicleHash = ENTITY::GET_ENTITY_MODEL(vehicleHandle);

	Vector3 vehiclePos = ENTITY::GET_ENTITY_COORDS(vehicleHandle, 0);

	// get gameobject dimensions
	Vector3 minCoords;	// min XYZ
	Vector3 maxCoords;  // max XYZ

	GAMEPLAY::GET_MODEL_DIMENSIONS(vehicleHash, &minCoords, &maxCoords);

	// reduce vehicle length as the box surrounding the vehicle has a slightly bigger length than the actual vehicle
	minCoords.y = minCoords.y * 0.96;
	maxCoords.y = maxCoords.y * 0.96;

	// generate the remaining corners of the 3D box surrounding the vehicle
	Vector3 minXminYmaxZCoords;	minXminYmaxZCoords.x = minCoords.x;	minXminYmaxZCoords.y = minCoords.y;	minXminYmaxZCoords.z = maxCoords.z;
	Vector3 maxXminYminZCoords;	maxXminYminZCoords.x = maxCoords.x;	maxXminYminZCoords.y = minCoords.y;	maxXminYminZCoords.z = minCoords.z;
	Vector3 maxXminYmaxZCoords;	maxXminYmaxZCoords.x = maxCoords.x;	maxXminYmaxZCoords.y = minCoords.y;	maxXminYmaxZCoords.z = maxCoords.z;
	Vector3 minXmaxYmaxZCoords;	minXmaxYmaxZCoords.x = minCoords.x;	minXmaxYmaxZCoords.y = maxCoords.y;	minXmaxYmaxZCoords.z = maxCoords.z;
	Vector3 maxXmaxYminZCoords;	maxXmaxYminZCoords.x = maxCoords.x;	maxXmaxYminZCoords.y = maxCoords.y;	maxXmaxYminZCoords.z = minCoords.z;
	Vector3 minXmaxYminZCoords;	minXmaxYminZCoords.x = minCoords.x;	minXmaxYminZCoords.y = maxCoords.y;	minXmaxYminZCoords.z = minCoords.z;

	std::vector<Vector3> corners = { minCoords, maxCoords, minXminYmaxZCoords, maxXminYminZCoords, maxXminYmaxZCoords, minXmaxYmaxZCoords, maxXmaxYminZCoords, minXmaxYminZCoords };

	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", "minCoords", std::to_string(minCoords.x) + " " + std::to_string(minCoords.y) + " " + std::to_string(minCoords.z));
	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", "maxCoords", std::to_string(maxCoords.x) + " " + std::to_string(maxCoords.y) + " " + std::to_string(maxCoords.z));
	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", "minXminYmaxZCoords", std::to_string(minXminYmaxZCoords.x) + " " + std::to_string(minXminYmaxZCoords.y) + " " + std::to_string(minXminYmaxZCoords.z));
	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", "maxXminYminZCoords", std::to_string(maxXminYminZCoords.x) + " " + std::to_string(maxXminYminZCoords.y) + " " + std::to_string(maxXminYminZCoords.z));
	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", "maxXminYmaxZCoords", std::to_string(maxXminYmaxZCoords.x) + " " + std::to_string(maxXminYmaxZCoords.y) + " " + std::to_string(maxXminYmaxZCoords.z));
	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", "minXmaxYmaxZCoords", std::to_string(minXmaxYmaxZCoords.x) + " " + std::to_string(minXmaxYmaxZCoords.y) + " " + std::to_string(minXmaxYmaxZCoords.z));
	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", "maxXmaxYminZCoords", std::to_string(maxXmaxYminZCoords.x) + " " + std::to_string(maxXmaxYminZCoords.y) + " " + std::to_string(maxXmaxYminZCoords.z));
	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", "minXmaxYminZCoords", std::to_string(minXmaxYminZCoords.x) + " " + std::to_string(minXmaxYminZCoords.y) + " " + std::to_string(minXmaxYminZCoords.z));

	// vehicle 3d box dimensions
	float vehicleDimWidth = maxCoords.x - minCoords.x;
	float vehicleDimHeight = maxCoords.y - minCoords.y;
	float vehicleDimLength = maxCoords.z - minCoords.z;

	Vector3 vehicleRotation = ENTITY::GET_ENTITY_ROTATION(vehicleHandle, 1);

	Vector3 vehicleLocalForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicleHandle);	 // local y vector

	float vehicleAngleZ = vehicleRotation.z;
	if (vehicleLocalForwardVector.y > 0)
		vehicleAngleZ = vehicleRotation.z;
	else
		vehicleAngleZ = 180 - vehicleRotation.z;

	/*if ((vehicleLocalForwardVector.x > 0 && vehicleLocalForwardVector.y < 0 && vehicleLocalForwardVector.z < 0) || 
		(vehicleLocalForwardVector.x > 0 && vehicleLocalForwardVector.y < 0 && vehicleLocalForwardVector.z > 0) ||
		(vehicleLocalForwardVector.x < 0 && vehicleLocalForwardVector.y < 0 && vehicleLocalForwardVector.z > 0))
		vehicleAngleZ = 180 - vehicleRotation.z;
	else
		vehicleAngleZ = vehicleRotation.z;*/

	Vector3 camRotation = CAM::GET_CAM_ROT(panoramicCam, 1);

	LogCustomInformation(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", "Camera rotation Z", std::to_string(camRotation.z));

	// convert [-180, 0] angles to their positive counterparts
	if (camRotation.z < 0)
		camRotation.z = 180 + (180 + camRotation.z);

	// TODO: needs better adjustments for accurate 2d bounding boxes
	// the vehicles rotation is in the intervals [0, 90], [90, 0], [0, -90], [-90, 0], so there's a need to 
	// convert the angle to their "real" values using the vehicle forward vector
	if (vehicleLocalForwardVector.x < 0 && vehicleLocalForwardVector.y < 0) // correto
	{
		// same
		if (vehicleLocalForwardVector.z < 0)
			vehicleAngleZ = vehicleAngleZ - camRotation.z + 90;
		else
			vehicleAngleZ = vehicleAngleZ - camRotation.z + 90;
	}
	else if (vehicleLocalForwardVector.x < 0 && vehicleLocalForwardVector.y > 0)
	{
		// same
		if (vehicleLocalForwardVector.z < 0)
			vehicleAngleZ = -vehicleAngleZ - camRotation.z - 90;
		else
			vehicleAngleZ = -vehicleAngleZ - camRotation.z - 90;
	}
	else if (vehicleLocalForwardVector.x > 0 && vehicleLocalForwardVector.y > 0)
	{
		// same
		if (vehicleLocalForwardVector.z < 0)
			vehicleAngleZ = -vehicleAngleZ - camRotation.z - 90;
		else
			vehicleAngleZ = -vehicleAngleZ - camRotation.z -90;// Not good
	}
	else if (vehicleLocalForwardVector.x > 0 && vehicleLocalForwardVector.y < 0)
	{
		// same
		if (vehicleLocalForwardVector.z < 0)
			vehicleAngleZ = vehicleAngleZ - camRotation.z;
		else
			vehicleAngleZ = vehicleAngleZ - camRotation.z;		// GOOD
	}


	LogVehicleCornersCoords(filePath + lidarFrameLogFilePathDebug, "RegisterVehicleInformation", vehicleHandle, vehicleAngleZ, corners, centerDot, vehiclePos, resolutionX, resolutionY, true);

	// find center of the box surrounding the veiculo through arithmetic average of the corners coordinates
	// Note: the vehicles position is not based on it's center 
	Vector3 vehicleTrueCenter = FindCenterCoordsOf3Dbox(corners, vehiclePos);	// position in world space

	// obtain min 3D corner and max 3D corner of the 3d box surrounding the car
	int truncated = 0;
	std::vector<Vector3> minMaxVectors = GetBestMinMaxCoords(corners, vehiclePos, truncated, minCoords, maxCoords, filePath);

	float minxProjected;
	float minyProjected;
	GRAPHICS::_WORLD3D_TO_SCREEN2D(minMaxVectors[0].x + vehiclePos.x, minMaxVectors[0].y + vehiclePos.y, minMaxVectors[0].z + vehiclePos.z, &minxProjected, &minyProjected);

	float maxxProjected;
	float maxyProjected;
	GRAPHICS::_WORLD3D_TO_SCREEN2D(minMaxVectors[1].x + vehiclePos.x, minMaxVectors[1].y + vehiclePos.y, minMaxVectors[1].z + vehiclePos.z, &maxxProjected, &maxyProjected);

	// vehicle center projection coordinates
	float minxVehicleProjected;
	float minyVehicleProjected;
	//GRAPHICS::_WORLD3D_TO_SCREEN2D(vehiclePos.x, vehiclePos.y, vehiclePos.z, &minxVehicleProjected, &minyVehicleProjected);
	GRAPHICS::_WORLD3D_TO_SCREEN2D(vehicleTrueCenter.x, vehicleTrueCenter.y, vehicleTrueCenter.z, &minxVehicleProjected, &minyVehicleProjected);
	
	// Values per vehicle: "Entity 
	//						| Hash 
	//						| minCornerX | minCornerY | minCornerZ | projMinCornerX | projMinCornerX 
	//						| maxCornerX | maxCornerY | maxCornerZ | projMaxCornerX | projMaxCornerY 
	//						| Posx | Posy | Posz | Rotx | Roty | Rotz 
	//						| projCenterX | projCenterY 
	//						| dimX | dimY | dimZ
	//						| objectType | truncated | 
	//						| forwardVecX | forwardVecY | forwardVecZ"

	// insert vehicle into the dictionary
	vehiclesLookupTable[vehicleHandle] = std::to_string(vehicleHandle) + " " + std::to_string(vehicleHash) + " "
		+ std::to_string(minMaxVectors[0].x) + " " + std::to_string(minMaxVectors[0].y) + " " + std::to_string(minMaxVectors[0].z) + " "
		+ std::to_string((int)(minxProjected * resolutionX * 1.5)) + " " + std::to_string((int)(minyProjected * resolutionY * 1.5)) + " "
		+ std::to_string(minMaxVectors[1].x) + " " + std::to_string(minMaxVectors[1].y) + " " + std::to_string(minMaxVectors[1].z) + " "
		+ std::to_string((int)(maxxProjected * resolutionX * 1.5)) + " " + std::to_string((int)(maxyProjected * resolutionY * 1.5)) + " "
		+ std::to_string(vehicleTrueCenter.x - centerDot.x) + " " + std::to_string(vehicleTrueCenter.y - centerDot.y) + " " + std::to_string(vehicleTrueCenter.z - centerDot.z) + " "
		+ std::to_string(vehicleRotation.x) + " " + std::to_string(vehicleRotation.y) + " " + std::to_string(vehicleRotation.z) + " "
		+ std::to_string((int)(minxVehicleProjected * resolutionX * 1.5)) + " " + std::to_string((int)(minyVehicleProjected * resolutionY * 1.5)) + " "
		+ std::to_string(vehicleDimWidth) + " " + std::to_string(vehicleDimHeight) + " " + std::to_string(vehicleDimLength) + " "
		+ "Car" + " " + std::to_string(truncated) + " "
		+ std::to_string(vehicleLocalForwardVector.x) + " " + std::to_string(vehicleLocalForwardVector.y) + " " + std::to_string(vehicleLocalForwardVector.z);

	//+ std::to_string(vehiclePos.x - centerDot.x) + " " + std::to_string(vehiclePos.y - centerDot.y) + " " + std::to_string(vehiclePos.z - centerDot.z) + " "
}

Vector3 rotate_point_around_z_axis(Vector3 point, float angle)
{
	Vector3 rotatedPoint;
	rotatedPoint.x = point.x*cos(angle*(PI / 180)) - point.y*sin(angle*(PI / 180));
	rotatedPoint.y = point.x*sin(angle*(PI / 180)) + point.y*cos(angle*(PI / 180));
	rotatedPoint.z = point.z;

	return rotatedPoint;
}

Vector3 rotate_point_around_x_axis(Vector3 point, float angle)
{
	Vector3 rotatedPoint;
	rotatedPoint.x = point.x;
	rotatedPoint.y = point.y*cos(angle*(PI / 180)) - point.z*sin(angle*(PI / 180));
	rotatedPoint.z = point.y*sin(angle*(PI / 180)) + point.z*cos(angle*(PI / 180));

	return rotatedPoint;
}

Vector3 rotate_point_around_y_axis(Vector3 point, float angle)
{
	Vector3 rotatedPoint;
	rotatedPoint.x = point.x*cos(angle*(PI / 180)) + point.z*sin(angle*(PI / 180));
	rotatedPoint.y = point.y;
	rotatedPoint.z = -point.x*sin(angle*(PI / 180)) + point.z*cos(angle*(PI / 180));

	return rotatedPoint;
}

void lidar(double horiFovMin, double horiFovMax, double vertFovMin, double vertFovMax, double horiStep, double vertStep, int range, std::string filePath, double error, int errorDist, std::ofstream& log)
{
	countFrames++;

	int maxIndexRowCounterForThisFrame = std::ceil(nVerticalSteps / nWorkloadFrames) * countFrames;

	if (countFrames == nWorkloadFrames)
	{
		maxIndexRowCounterForThisFrame = maxIndexRowCounterForThisFrame + (nVerticalSteps - maxIndexRowCounterForThisFrame);
	}

	try {
		log.open(lidarLogFilePath, std::ios_base::app);
		for (double x = xValue; x <= vertFovMax; x += vertStep)	// -15 to 12, in steps of 0.5 => (15+12)/0.5 = 74 vertical points for each horizontal angle
		{
			if (indexRowCounter == (maxIndexRowCounterForThisFrame + 1))
			{
				// xValue stores the x value to be calculated in the next frame
				xValue = x;
				break;
			}

			pointsPerVerticalStep[indexRowCounter] = 0;

			int indexColumnCounter = 0;

			for (double z = horiFovMin; z <= horiFovMax; z += horiStep)	 // 0 to 360, in steps of 0.5 => 360/0.5 = 720 horizontal/circle points
			{
				ray result = angleOffsetRaycast(x, z, range);

				RegisterVehicleInformation(result.hitEntityHandle, result.entityTypeName, filePath);

				// register the distance between the collision point and the ray origin
				pointsMatrix[indexRowCounter][indexColumnCounter] = result.hitCoordinates;

				// introduce error to the points
				Vector3 xyzError;
				introduceError(&xyzError, result.hitCoordinates.x - centerDot.x, result.hitCoordinates.y - centerDot.y, result.hitCoordinates.z - centerDot.z, error, errorDist, range, dist_vector, error_vector);

				pointsWithErrorMatrix[indexRowCounter][indexColumnCounter] = xyzError;

				// gameobject type (Background, vehicle, pedestrian, props)
				labels[indexRowCounter][indexColumnCounter] = result.entityTypeId;

				// gameobject instance entity id
				labelsDetailed[indexRowCounter][indexColumnCounter] = result.hitEntityHandle;

				// a raycast only outputs a point when it hits something
				pointsPerVerticalStep[indexRowCounter]++; 

				indexColumnCounter++;
			}

			indexRowCounter++;
		}

		// write scene vehicles dimensions to file
		std::ofstream sampleVehicleDimFileW;
		sampleVehicleDimFileW.open(filePath + vehiclesInSampleFilename);

		for (const auto &myPair : vehiclesLookupTable) {
			sampleVehicleDimFileW << myPair.second + "\n";
		}

		sampleVehicleDimFileW.close();

		log.close();
	}
	catch (std::exception &e)
	{
		return;
	}
}

void PostLidarScanProcessing(std::string filePath)
{
	float cam_rotz;

	Vector3 playerCurRot = ENTITY::GET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), 0);
	
	//Set clear weather
	GAMEPLAY::CLEAR_OVERRIDE_WEATHER();
	GAMEPLAY::SET_OVERRIDE_WEATHER("CLEAR");

	//Set time to midnight
	TIME::SET_CLOCK_TIME(0, 0, 0);
	WAIT(100);

	//Iterate 2 times, with X and Y translation
	//For this one, i assumes values of -1 and 1, to make X rotations
	for (int i = -1; i < 2; i += 2) {
		//Chaging the distance of camera to fit the kitti dataset
		CAM::ATTACH_CAM_TO_ENTITY(panoramicCam, PLAYER::PLAYER_PED_ID(), (i * 0.27) + 0.1, 6, raycastHeightParam - halfCharacterHeight, 1);

		cam_rotz = playerCurRot.z + 180 - (i * 5);

		CAM::SET_CAM_ROT(panoramicCam, 0, 0, cam_rotz, 1);
		WAIT(200);

		//Save screenshot
		std::string filename = filePath + "_Camera_Print_Night_" + std::to_string(i+1) + ".bmp";
		SaveScreenshot(filename.c_str());
	}

	//Set cloudy weather
	GAMEPLAY::CLEAR_OVERRIDE_WEATHER();
	GAMEPLAY::SET_OVERRIDE_WEATHER("OVERCAST");

	//Set time to noon
	TIME::SET_CLOCK_TIME(12, 0, 0);

	//Rotate camera 360 degrees and take screenshots
	WAIT(80);
	//For this one, i assumes values of -1 and 1, to make X rotations
	for (int i = -1; i < 2; i+=2) {
		//Rotate camera

		CAM::ATTACH_CAM_TO_ENTITY(panoramicCam, PLAYER::PLAYER_PED_ID(), (i * 0.27) + 0.1 , 6, raycastHeightParam - halfCharacterHeight, 1);

		//Angle to face the car
		cam_rotz = playerCurRot.z + 180 - (i * 5);
		CAM::SET_CAM_ROT(panoramicCam, 0, 0, cam_rotz, 1);
		WAIT(200);

		//Save screenshot
		std::string filename = filePath + "_Camera_Print_Cloudy_" + std::to_string(i+1) + ".bmp";
		SaveScreenshot(filename.c_str());
	}

	//Set clear weather
	GAMEPLAY::CLEAR_OVERRIDE_WEATHER();
	GAMEPLAY::SET_OVERRIDE_WEATHER("CLEAR");

	//reset camera
	CAM::ATTACH_CAM_TO_ENTITY(panoramicCam, PLAYER::PLAYER_PED_ID(), 0, 6, raycastHeightParam - halfCharacterHeight, 1);


	//Iterate 2 times, with X and Y translation
	//For this one, i assumes values of -1 and 1, to make X rotations
	for (int i = -1; i < 2; i += 2) {

		CAM::ATTACH_CAM_TO_ENTITY(panoramicCam, PLAYER::PLAYER_PED_ID(), (i * 0.27) + 0.1, 6, raycastHeightParam - halfCharacterHeight, 1);

		//Rotate camera
		cam_rotz = playerCurRot.z + 180 - (i * 5);
		CAM::SET_CAM_ROT(panoramicCam, 0, 0, cam_rotz, 1);
		WAIT(200);

		//Save screenshot
		std::string filename = filePath + "_Camera_Print_Day_" + std::to_string(i+1) + ".bmp";
		SaveScreenshot(filename.c_str());

		// Iterate through every point, determine if it's present in the current FoV, if it is project it onto the picture and assign the screenX, screenY and FoV id to the respective point
		//log << "Projecting 3D points to 2D view...";

		for (int t = 0; t < nVerticalSteps; t++)
		{
			for (int j = 0; j < nHorizontalSteps; j++)
			{
				Vector3 voxel = pointsMatrix[t][j];

				//Get screen coordinates of 3D voxels
				GRAPHICS::_WORLD3D_TO_SCREEN2D(voxel.x, voxel.y, voxel.z, &x2d, &y2d);

				//Get screen coordinates of 3D voxels w/ error
				GRAPHICS::_WORLD3D_TO_SCREEN2D(pointsWithErrorMatrix[t][j].x + centerDot.x, pointsWithErrorMatrix[t][j].y + centerDot.y, pointsWithErrorMatrix[t][j].z + centerDot.z, &err_x2d, &err_y2d);

				if (x2d != -1 || y2d != -1) {
					if (pointsProjectedMatrix[t][j].stateSet == false) // if point hasn't already been colored
					{
						pointsProjectedMatrix[t][j].stateSet = true;
						pointsProjectedMatrix[t][j].pictureId = i;
						pointsProjectedMatrix[t][j].screenCoordX = int(x2d * resolutionX * 1.5);
						pointsProjectedMatrix[t][j].screenCoordY = int(y2d * resolutionY * 1.5);
					}
				}

				if (err_x2d > -1 || err_y2d > -1) {
					if (pointsProjectedWithErrorMatrix[t][j].stateSet == false) // if point hasn't already been colored
					{
						pointsProjectedWithErrorMatrix[t][j].stateSet = true;
						pointsProjectedWithErrorMatrix[t][j].pictureId = i;
						pointsProjectedWithErrorMatrix[t][j].screenCoordX = int(err_x2d * resolutionX * 1.5);
						pointsProjectedWithErrorMatrix[t][j].screenCoordY = int(err_y2d * resolutionY * 1.5);
					}
				}
			}
		}
		//log << "Done.\n";
	}
	
	std::string fileOutputLines = "";
	std::string fileOutputPointsLines = "";
	std::string fileOutputErrorLines = "";
	std::string fileOutputErrorPointsLines = "";
	std::string labelsFileStreamWLines = "";
	std::string labelsDetailedFileStreamWLines = "";

	int countValidPoints = 0;
	// fill LiDAR_PointCloud_error.txt
	for (int i = 0; i < nVerticalSteps; i++)
	{
		for (int j = 0; j < pointsPerVerticalStep[i]; j++)
		{
			if (!(pointsMatrix[i][j].x == 0.0 && pointsMatrix[i][j].y == 0.0 && pointsMatrix[i][j].z == 0.0))	// if the point is (0, 0, 0), it means that it didn't collide with anything
			{
				countValidPoints++;

				// vertexData; fill point cloud .ply
				fileOutputLines += std::to_string(pointsMatrix[i][j].x - centerDot.x) + " " + std::to_string(pointsMatrix[i][j].y - centerDot.y) + " " + std::to_string(pointsMatrix[i][j].z - centerDot.z) + "\n";

				// fill LiDAR_PointCloud_points.txt
				fileOutputPointsLines += std::to_string(pointsMatrix[i][j].x - centerDot.x) + " " + std::to_string(pointsMatrix[i][j].y - centerDot.y) + " " + std::to_string(pointsMatrix[i][j].z - centerDot.z) + " " + std::to_string(pointsProjectedMatrix[i][j].screenCoordX) + " " + std::to_string(pointsProjectedMatrix[i][j].screenCoordY) + " " + std::to_string(pointsProjectedMatrix[i][j].pictureId) + "\n";

				// fill point cloud with errors .ply
				fileOutputErrorLines += std::to_string(pointsWithErrorMatrix[i][j].x) + " " + std::to_string(pointsWithErrorMatrix[i][j].y) + " " + std::to_string(pointsWithErrorMatrix[i][j].z) + "\n";

				// fill LiDAR_PointCloud_error.txt
				fileOutputErrorPointsLines += std::to_string(pointsWithErrorMatrix[i][j].x) + " " + std::to_string(pointsWithErrorMatrix[i][j].y) + " " + std::to_string(pointsWithErrorMatrix[i][j].z) + " " + std::to_string(pointsProjectedWithErrorMatrix[i][j].screenCoordX) + " " + std::to_string(pointsProjectedWithErrorMatrix[i][j].screenCoordY) + " " + std::to_string(pointsProjectedWithErrorMatrix[i][j].pictureId) + "\n";

				// fill labels txt
				labelsFileStreamWLines += std::to_string(labels[i][j]) + "\n";

				// fill labels detailed txt
				labelsDetailedFileStreamWLines += std::to_string(labelsDetailed[i][j]) + "\n";
			}
		}
	}

	fileOutput << "ply\nformat ascii 1.0\nelement vertex " + std::to_string(countValidPoints) + "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
	fileOutputError << "ply\nformat ascii 1.0\nelement vertex " + std::to_string(countValidPoints) + "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";

	fileOutput << fileOutputLines;
	fileOutputPoints << fileOutputPointsLines;
	fileOutputError << fileOutputErrorLines;
	fileOutputErrorPoints << fileOutputErrorPointsLines;
	labelsFileStreamW << labelsFileStreamWLines;
	labelsDetailedFileStreamW << labelsDetailedFileStreamWLines;

	Vector3 playerCurrentRot = ENTITY::GET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), 0);
	Vector3 playerForwardDirection = ENTITY::GET_ENTITY_FORWARD_VECTOR(PLAYER::PLAYER_PED_ID());

	// write to a file inside a sample, the rotation the the character is facing
	std::ofstream sampleCharRotFileW;
	sampleCharRotFileW.open(filePath + playerRotationInSampleFilename);
	sampleCharRotFileW << std::to_string(playerCurrentRot.x) + " " + std::to_string(playerCurrentRot.y) + " " + std::to_string(playerCurrentRot.z) + " " + std::to_string(playerForwardDirection.x) + " " + std::to_string(playerForwardDirection.y) + " " + std::to_string(playerForwardDirection.z);
	sampleCharRotFileW.close();

	GAMEPLAY::SET_GAME_PAUSED(false);
	TIME::PAUSE_CLOCK(false);
	WAIT(10);

	fileOutput.close();
	fileOutputPoints.close();
	fileOutputError.close();
	fileOutputErrorPoints.close();
	labelsFileStreamW.close(); // close labels file
	labelsDetailedFileStreamW.close();

	//Restore original camera
	CAM::RENDER_SCRIPT_CAMS(0, 0, 0, 1, 0);
	CAM::DESTROY_CAM(panoramicCam, 1);

	//Unpause game
	GAMEPLAY::SET_TIME_SCALE(NormalSpeed);

	//Restore HUD and Radar
	UI::DISPLAY_RADAR(true);
	UI::DISPLAY_HUD(true);
	ENTITY::RESET_ENTITY_ALPHA(PLAYER::PLAYER_PED_ID());		// restore alphalevel to make character visible again

	//log.close();
}
