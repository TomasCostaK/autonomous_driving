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

#pragma comment(lib, "gdiplus.lib")

#pragma warning(disable : 4244 4305) // double <-> float conversions

#define StopSpeed 0.00000000001f
#define NormalSpeed 1.0f

static std::default_random_engine generator;
static std::uniform_real_distribution<double> distribution(-1.0, 1.0);

using namespace Gdiplus;

/**
	File path global variables, include files for logging, lidar configurations, jitter generation,
	point cloud labeling, stored positions and rotations of vehicle route landmarks.
**/
std::string lidarParentDir = "LiDAR GTA V";
std::string lidarLogFilePath = lidarParentDir + "/log.txt";
std::string lidarCfgFilePath = lidarParentDir + "/LiDAR GTA V.cfg";
std::string lidarErrorDistFilePath = lidarParentDir + "/dist_error.csv";
std::string lidarPointLabelsFilePath = lidarParentDir + "/pointcloudLabels.txt";
std::string routeFilePath = lidarParentDir + "/_positionsDB.txt";
std::string playerRotationsFilename = lidarParentDir + "/_rotationsDB.txt";

/**
	Global output streams for storing pointcloud segmentation using 4 classes (background, vehicle,
	human and animals, game props) and segmenting by gameobject id (detailed).
**/
std::ofstream labelsFileStreamW;
std::ofstream labelsDetailedFileStreamW;

std::ofstream playerRotationsStreamW;				// Output stream for storing player rotation information for each landmark of the traced vehicle route.


// Number of currently recorded positions (initiated by pressing the F3 key).
int positionsCounter = 0;

/**
	State variable for when a new lidar scan is going to happen.
	It is used to enable a sequence of lidar scans separated by a set time interval.
**/
bool takeSnap = false;

bool playerTeleported = false;							// State variable set when the player character has teleported. Makes it possible to impose a waiting time for the game assets to load.
bool displayNotice = true;								// Display a info note when the F5 key is pressed for the first time.
bool isRecordingPositions = false;						// State variable set when position recording is taking place (F3 key).
bool isAutoScanning = false;							// State variable set when the automatic LiDAR scanning process is in progress (initiated by F5 key).
bool isWaitAfterTeleportFinished = false;
bool scanLive = false;									// State variable set when a lidar scan is being executed
bool lidarScanPrep = false;

/**
	State variable set when the user recorded positions in the current instance of the game.
	When true, new recorded positions will be appended to previous ones.
**/
bool haveRecordedPositions = false;

bool hasStartedAndStoppedAutoScan = false;			// State variable that stores if the user has already performed (and interrupted) the automatic lidar scanning for the first time in the current game instance.

/**
	Lidar height value definition.
**/
float halfCharacterHeight = 1.12;					// Fixed value. GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS() gives the player's position (at the middle of the character model, and not at foot level); in meters.
float raycastHeightParam = 2.40;					// This variable is used to adjust the height at which to position the origin of the lidar coordinate system. 
													// The 2.40 meters in-game corresponds to 1.70m in real life (it already takes into account the halfCharacterHeight value).

/**
	Automatic lidar scanning global variables.
**/

// position recording
float secondsBetweenRecordings = 2;					// seconds between position recording

// lidar scanning
int secondsToWaitAfterTeleport = 5;



int secondsBeforeStartingLidarScan = 2;				// in order for the scanning start to not be sudden
//float secondsBetweenLidarSnapshots = 14;			// taking into account the time that the teleport, lidar scanning and snapshots take


int positionsFileNumberOfLines = -1;				// number of lines of the file with the route
int snapshotsCounter = 0;							// number of lidar scans completed



// current player position for lidar scanning
Vector3 playerPos;


/* Stuff for lidar auto scanning process */

std::vector<double> dist_vector, error_vector;
Vector3 centerDot;
int resolutionX, resolutionY;
Vector3 rot;
Cam panoramicCam;
std::ofstream fileOutput, fileOutputPoints, fileOutputError, fileOutputErrorPoints, fileOutputErrorDist;
float x2d, y2d, err_x2d, err_y2d;

std::vector<std::vector<Vector3>> pointsMatrix;
std::vector<std::vector<Vector3>> pointsWithErrorMatrix;
std::vector<std::vector<ProjectedPointData>> pointsProjectedMatrix;
std::vector<std::vector<ProjectedPointData>> pointsProjectedWithErrorMatrix;

std::vector<std::vector<int>> labels;
std::vector<std::vector<int>> labelsDetailed;

std::vector<int> pointsPerVerticalStep;

int nHorizontalSteps;
int nVerticalSteps;

int no_of_cols;
int no_of_rows;
int initial_value;

int nWorkloadFrames = 4; // numero par
int countFrames;
int indexRowCounter = 0;
//int indexColumnCounter = 0;
double xValue;
//double zValue;

void ScriptMain()
{
	srand(GetTickCount());

	// stream for the positionsDB text file for writing
	std::ofstream positionsDBFileW;

	// stream for the positionsDB text file for reading
	std::ifstream positionsDBFileR;

	std::ifstream playerRotationsStreamR;

	auto start_time_for_collecting_positions = std::chrono::high_resolution_clock::now();

	auto start_time_after_teleport = std::chrono::high_resolution_clock::now();

	//auto start_time_for_lidar_scanning = std::chrono::high_resolution_clock::now();

	// event handling loop
	while (true)
	{
		// keyboard commands information
		if (IsKeyJustUp(VK_F1))
		{
			notificationOnLeft("- F2: teleport to the route starting position\n- F3: start/stop recording player position\n- F4: scripthook V menu\n- F5: start/stop automatic scanning");
		}

		// teleport the character to the start position of the route, or the landmark from where the automatic scanning was interrupted/stopped
		if (IsKeyJustUp(VK_F2) && !isAutoScanning && !isRecordingPositions)
		{
			try {
				std::ifstream positionsFileRforTeleportation;
				positionsFileRforTeleportation.open(routeFilePath);

				std::ifstream rotationsFileRforTeleportation;
				rotationsFileRforTeleportation.open(playerRotationsFilename);

				// go to the line in each file for the next landmark to teleport to.
				GotoLineInPositionsDBFile(positionsFileRforTeleportation, snapshotsCounter + 1);
				GotoLineInPositionsDBFile(rotationsFileRforTeleportation, snapshotsCounter + 1);

				// transport the player to the route position 
				// this way the assets can be loaded before the lidar scanning starts
				std::string xStr, yStr, zStr;
				float xRot, yRot, zRot;// , wRot;

				if (positionsFileRforTeleportation >> xStr >> yStr >> zStr && rotationsFileRforTeleportation >> xRot >> yRot >> zRot)
				{
					Vector3 playerPosDest;
					playerPosDest.x = atof(xStr.c_str());
					playerPosDest.y = atof(yStr.c_str());
					playerPosDest.z = atof(zStr.c_str());

					PED::SET_PED_COORDS_NO_GANG(PLAYER::PLAYER_PED_ID(), playerPosDest.x, playerPosDest.y, playerPosDest.z);
					ENTITY::SET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), xRot, yRot, zRot, 0, 0);

					// needed to ensure that the character doesn't automatically rotate to the last input rotation.
					PED::SET_PED_COORDS_NO_GANG(PLAYER::PLAYER_PED_ID(), playerPosDest.x, playerPosDest.y, playerPosDest.z);
					ENTITY::SET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), xRot, yRot, zRot, 0, 0);
				}

				rotationsFileRforTeleportation.close();
				positionsFileRforTeleportation.close();
			}
			catch (std::exception &e)
			{
				notificationOnLeft(e.what());
				return;
			}
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
				// get player position + offset
				Vector3 playerCurrentPos = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(PLAYER::PLAYER_PED_ID(), 0, 0, -halfCharacterHeight);

				// write current player position to the text file
				positionsDBFileW << std::to_string(playerCurrentPos.x) + " " + std::to_string(playerCurrentPos.y) + " " + std::to_string(playerCurrentPos.z) + "\n";

				float rotx, roty, rotz, rotw;
				///ENTITY::GET_ENTITY_QUATERNION(PLAYER::PLAYER_PED_ID(), &rotx, &roty, &rotz, &rotw);
				Vector3 rot = ENTITY::GET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), 0);

				rotx = rot.x;
				roty = rot.y;
				rotz = rot.z;

				///playerRotationsStreamW << std::to_string(rotx) + " " + std::to_string(roty) + " " + std::to_string(rotz) + " " + std::to_string(rotw) + "\n";
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

			if (CheckNumberOfLinesInFile(routeFilePath) - snapshotsCounter > 0) // if not all the positions in the file were scanned, continue with the scanning of the rest of the positions
			{
				if (displayNotice)
				{
					notificationOnLeft("Reminder: In order for the program to successfully write new data, make sure that there aren't any folders created by previous scannings.\n\nPress F5 again to start!");
					displayNotice = false;
				}
				else
				{
					try
					{
						if (!isAutoScanning)
						{
							//start_time_for_lidar_scanning = std::chrono::high_resolution_clock::now();

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
								notificationOnLeft("Lidar scanning restarting in " + std::to_string((int)(secondsToWaitAfterTeleport)) + " seconds" + "\n\nRemaining positions: " + std::to_string(positionsFileNumberOfLines - snapshotsCounter));
								GotoLineInPositionsDBFile(positionsDBFileR, snapshotsCounter + 1);
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
			//auto current_time = std::chrono::high_resolution_clock::now();

			//double elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_for_lidar_scanning).count();

			//std::string xStr, yStr, zStr;
			float xPos, yPos, zPos;
			float xRot, yRot, zRot;
			// get next position to teleport to
			///if (positionsDBFileR >> xStr >> yStr >> zStr && playerRotationsStreamR >> xRot >> yRot >> zRot >> wRot)
			if (positionsDBFileR >> xPos >> yPos >> zPos && playerRotationsStreamR >> xRot >> yRot >> zRot)
			{
				Vector3 playerCurrentPos = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(PLAYER::PLAYER_PED_ID(), 0, 0, -halfCharacterHeight);

				Vector3 playerCurrentRot = ENTITY::GET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), 0);

				/*playerPos.x = atof(xStr.c_str());
				playerPos.y = atof(yStr.c_str());
				playerPos.z = atof(zStr.c_str());
				playerRot.x = atof(xRot.c_str());
				playerRot.y = atof(yRot.c_str());
				playerRot.z = atof(zRot.c_str());*/

				// update global variable
				playerPos.x = xPos;
				playerPos.y = yPos;
				playerPos.z = zPos;

				if (xPos != playerCurrentPos.x || yPos != playerCurrentPos.y || zPos != playerCurrentPos.z)
				{
					PED::SET_PED_COORDS_NO_GANG(PLAYER::PLAYER_PED_ID(), playerPos.x, playerPos.y, playerPos.z);
					// reorient the player in order for the pictures to start to be taken from the view of the player
					///ENTITY::SET_ENTITY_QUATERNION(PLAYER::PLAYER_PED_ID(), xRot, yRot, zRot, wRot);
					ENTITY::SET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), xRot, yRot, zRot, 0, 0);
				}

				if (xRot != playerCurrentRot.x || yRot != playerCurrentRot.y || zRot != playerCurrentRot.z)
				{
					// in order to prevent the character from automatically rotating to the last input direction, repeat the teleport and reorientation
					PED::SET_PED_COORDS_NO_GANG(PLAYER::PLAYER_PED_ID(), playerPos.x, playerPos.y, playerPos.z);
					///ENTITY::SET_ENTITY_QUATERNION(PLAYER::PLAYER_PED_ID(), xRot, yRot, zRot, wRot);
					ENTITY::SET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), xRot, yRot, zRot, 0, 0);
				}

				playerTeleported = true;
				isWaitAfterTeleportFinished = false;
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

		if (isAutoScanning && !playerTeleported && !isWaitAfterTeleportFinished)
		{
			auto currentTime = std::chrono::high_resolution_clock::now();

			double elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - start_time_after_teleport).count();

			if (elapsedTime > secondsToWaitAfterTeleport)
			{
				takeSnap = true;
				isWaitAfterTeleportFinished = true;
			}
		}

		// output stream for writing log information into the log.txt file
		std::ofstream log;
		// press F2 to capture environment images and point cloud
		if (takeSnap)
		{
			// reset
			takeSnap = false;
			if (isAutoScanning) // in order to not increase the counter when doing manual snapshots
				snapshotsCounter++;

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
				std::string newfolder = "LiDAR GTA V/" + filename + std::to_string(snapshotsCounter);
				_mkdir(newfolder.c_str());

				// prepare for lidar scan
				SetupGameForLidarScan(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], newfolder + "/" + filename);

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
			std::string newfolder = "LiDAR GTA V/" + filename + std::to_string(snapshotsCounter);

			lidar(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], rangeRay, newfolder + "/" + filename, error, errorDist, log);

			if (countFrames == nWorkloadFrames)
			{
				scanLive = false;
				lidarScanPrep = false;
				PostLidarScanProcessing(newfolder + "/" + filename);

				int percentageComplete = ((float)snapshotsCounter) / ((float)positionsFileNumberOfLines) * 100;
				notificationOnLeft("Snapshots taken: " + std::to_string(snapshotsCounter) + "\n\nCompleted: " + std::to_string(percentageComplete) + "%");
			}
		}

		WAIT(0);
	}
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
	double rotationX = (/*rot.x + */angleOffsetX) * (M_PI / 180.0);
	double rotationZ = (/*rot.z + */angleOffsetZ) * (M_PI / 180.0);
	double multiplyXY = abs(cos(rotationX));
	Vector3 direction;
	direction.x = sin(rotationZ) * multiplyXY * -1;
	direction.y = cos(rotationZ) * multiplyXY;
	direction.z = sin(rotationX);

	Vector3 raycastCenterPos;
	raycastCenterPos.x = playerPos.x;
	raycastCenterPos.y = playerPos.y;
	raycastCenterPos.z = playerPos.z + raycastHeightParam;

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

// this function is pnly called once at the beginning of each lidar scan
void SetupGameForLidarScan(double horiFovMin, double horiFovMax, double vertFovMin, double vertFovMax, double horiStep, double vertStep, std::string filePath)
{
	readErrorFile(dist_vector, error_vector);

	centerDot.x = playerPos.x;
	centerDot.y = playerPos.y;
	centerDot.z = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(PLAYER::PLAYER_PED_ID(), 0, 0, raycastHeightParam - halfCharacterHeight).z;

	GRAPHICS::GET_SCREEN_RESOLUTION(&resolutionX, &resolutionY);

	//Set camera on top of player
	//log << "Setting up camera...";
	rot = CAM::GET_GAMEPLAY_CAM_ROT(0);
	panoramicCam = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", 1);
	CAM::SET_CAM_FOV(panoramicCam, 90);
	CAM::SET_CAM_ROT(panoramicCam, 0, 0, rot.z, 2);
	CAM::ATTACH_CAM_TO_ENTITY(panoramicCam, PLAYER::PLAYER_PED_ID(), 0, 0, raycastHeightParam - halfCharacterHeight, 1);
	CAM::RENDER_SCRIPT_CAMS(1, 0, 0, 1, 0);
	CAM::SET_CAM_ACTIVE(panoramicCam, true);
	WAIT(50);

	//int vertexCount = (horiFovMax - horiFovMin) * (1 / horiStep) * (vertFovMax - vertFovMin) * (1 / vertStep);		// theoretical vertex count (if all raycasts intercept with an object)

	//log << " Done.\n";

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

	//Take 3D point cloud
	//log << "Taking 3D point cloud...\n";

	// paralel processing for obtaining all point cloud points
	nHorizontalSteps = (horiFovMax - horiFovMin) / horiStep;
	nVerticalSteps = (vertFovMax - vertFovMin) / vertStep;

	no_of_cols = nHorizontalSteps;
	no_of_rows = nVerticalSteps;
	initial_value = 0;

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
	//zValue = horiFovMin;

	countFrames = 0;
	indexRowCounter = 0;

	lidarScanPrep = true;
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
				float cam_rotz = rot.z + z;

				ray result = angleOffsetRaycast(x, cam_rotz, range);

				// if the ray collided with something, register the distance between the collition point and the ray origin
				if (!(result.hitCoordinates.x == 0. && result.hitCoordinates.y == 0. && result.hitCoordinates.z == 0.))
				{
					pointsMatrix[indexRowCounter][indexColumnCounter] = result.hitCoordinates;

					//Introduce error in voxels
					Vector3 xyzError;
					introduceError(&xyzError, result.hitCoordinates.x - centerDot.x, result.hitCoordinates.y - centerDot.y, result.hitCoordinates.z - centerDot.z, error, errorDist, range, dist_vector, error_vector);

					pointsWithErrorMatrix[indexRowCounter][indexColumnCounter] = xyzError;

					// prints the object id. Each model/mesh of the game has its own id
					labels[indexRowCounter][indexColumnCounter] = result.entityTypeId;
					labelsDetailed[indexRowCounter][indexColumnCounter] = result.hitEntityHandle;

					pointsPerVerticalStep[indexRowCounter]++; // a raycast only outputs a point when it hits something
				}

				indexColumnCounter++;
			}

			indexRowCounter++;
		}
	}
	catch (std::exception &e)
	{
		return;
	}
}

void PostLidarScanProcessing(std::string filePath)
{
	int k = 0; // counter for the number of points sampled
	// count how many points the generated point cloud has
	for (int i = 0; i < nVerticalSteps; i++)
	{
		k += pointsPerVerticalStep[i];
	}

	//log << std::to_string(k) + " points filled.\nDone.\n";

	float cam_rotz;

	//Set clear weather
	GAMEPLAY::CLEAR_OVERRIDE_WEATHER();
	GAMEPLAY::SET_OVERRIDE_WEATHER("CLEAR");
	//Set time to midnight
	TIME::SET_CLOCK_TIME(0, 0, 0);
	WAIT(100);
	//Rotate camera 360 degrees and take screenshots
	for (int i = 0; i < 3; i++) {
		//Rotate camera
		cam_rotz = rot.z + i * 120;
		CAM::SET_CAM_ROT(panoramicCam, 0, 0, cam_rotz, 2);
		WAIT(200);

		//Save screenshot
		std::string filename = filePath + "_Camera_Print_Night_" + std::to_string(i) + ".bmp";
		SaveScreenshot(filename.c_str());
	}

	//Set clear weather
	GAMEPLAY::CLEAR_OVERRIDE_WEATHER();
	GAMEPLAY::SET_OVERRIDE_WEATHER("OVERCAST");
	//Set time to noon
	TIME::SET_CLOCK_TIME(12, 0, 0);
	//Rotate camera 360 degrees and take screenshots
	WAIT(100);
	for (int i = 0; i < 3; i++) {
		//Rotate camera
		cam_rotz = rot.z + i * 120;
		CAM::SET_CAM_ROT(panoramicCam, 0, 0, cam_rotz, 2);
		WAIT(200);

		//Save screenshot
		std::string filename = filePath + "_Camera_Print_Cloudy_" + std::to_string(i) + ".bmp";
		SaveScreenshot(filename.c_str());
	}

	//Set clear weather
	GAMEPLAY::CLEAR_OVERRIDE_WEATHER();
	GAMEPLAY::SET_OVERRIDE_WEATHER("CLEAR");

	for (int i = 0; i < 3; i++) {
		//Rotate camera
		cam_rotz = rot.z + i * 120;
		CAM::SET_CAM_ROT(panoramicCam, 0, 0, cam_rotz, 2);
		WAIT(200);

		//Save screenshot
		std::string filename = filePath + "_Camera_Print_Day_" + std::to_string(i) + ".bmp";
		SaveScreenshot(filename.c_str());

		// Iterate through every point, determine if it's present in the current FoV, if it is project it onto the picture and assign the screenX, screenY and FoV id to the respective point
		// This information is stored in the files LiDAR_PointCloud_error.txt and LiDAR_PointCloud_points.txt, to allow an external python script to correctly color the uncolored point cloud (based upon the 3 pictures taken)
		//log << "Converting 3D to 2D...";

		for (int k = 0; k < nVerticalSteps; k++)
		{
			for (int j = 0; j < nHorizontalSteps; j++)
			{
				Vector3 voxel = pointsMatrix[k][j];

				//Get screen coordinates of 3D voxels
				GRAPHICS::_WORLD3D_TO_SCREEN2D(voxel.x, voxel.y, voxel.z, &x2d, &y2d);


				//Get screen coordinates of 3D voxels w/ error
				GRAPHICS::_WORLD3D_TO_SCREEN2D(pointsWithErrorMatrix[k][j].x + centerDot.x, pointsWithErrorMatrix[k][j].y + centerDot.y, pointsWithErrorMatrix[k][j].z + centerDot.z, &err_x2d, &err_y2d);

				if (x2d != -1 || y2d != -1) {
					if (pointsProjectedMatrix[k][j].stateSet == false) // if point hasn't already been colored
					{
						pointsProjectedMatrix[k][j].stateSet = true;
						pointsProjectedMatrix[k][j].pictureId = i;
						pointsProjectedMatrix[k][j].screenCoordX = int(x2d * resolutionX * 1.5);
						pointsProjectedMatrix[k][j].screenCoordY = int(y2d * resolutionY * 1.5);
					}
				}

				if (err_x2d > -1 || err_y2d > -1) {
					if (pointsProjectedWithErrorMatrix[k][j].stateSet == false) // if point hasn't already been colored
					{
						pointsProjectedWithErrorMatrix[k][j].stateSet = true;
						pointsProjectedWithErrorMatrix[k][j].pictureId = i;
						pointsProjectedWithErrorMatrix[k][j].screenCoordX = int(err_x2d * resolutionX * 1.5);
						pointsProjectedWithErrorMatrix[k][j].screenCoordY = int(err_y2d * resolutionY * 1.5);
					}
				}
			}
		}
		//log << "Done.\n";
	}

	//log << "Deleting array...";
	//log << "Done.\n";
	//log << "Writing to files...";
	fileOutput << "ply\nformat ascii 1.0\nelement vertex " + std::to_string(k) + "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
	fileOutputError << "ply\nformat ascii 1.0\nelement vertex " + std::to_string(k) + "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";

	// fill LiDAR_PointCloud_error.txt
	for (int i = 0; i < nVerticalSteps; i++)
	{
		for (int j = 0; j < pointsPerVerticalStep[i]; j++)
		{
			// vertexData; fill point cloud .ply
			fileOutput << std::to_string(pointsMatrix[i][j].x - centerDot.x) + " " + std::to_string(pointsMatrix[i][j].y - centerDot.y) + " " + std::to_string(pointsMatrix[i][j].z - centerDot.z) + "\n";

			// fill LiDAR_PointCloud_points.txt
			fileOutputPoints << std::to_string(pointsMatrix[i][j].x - centerDot.x) + " " + std::to_string(pointsMatrix[i][j].y - centerDot.y) + " " + std::to_string(pointsMatrix[i][j].z - centerDot.z) + " " + std::to_string(pointsProjectedMatrix[i][j].screenCoordX) + " " + std::to_string(pointsProjectedMatrix[i][j].screenCoordY) + " " + std::to_string(pointsProjectedMatrix[i][j].pictureId) + "\n";

			// fill point cloud with errors .ply
			fileOutputError << std::to_string(pointsWithErrorMatrix[i][j].x) + " " + std::to_string(pointsWithErrorMatrix[i][j].y) + " " + std::to_string(pointsWithErrorMatrix[i][j].z) + "\n";

			// fill LiDAR_PointCloud_error.txt
			fileOutputErrorPoints << std::to_string(pointsWithErrorMatrix[i][j].x) + " " + std::to_string(pointsWithErrorMatrix[i][j].y) + " " + std::to_string(pointsWithErrorMatrix[i][j].z) + " " + std::to_string(pointsProjectedWithErrorMatrix[i][j].screenCoordX) + " " + std::to_string(pointsProjectedWithErrorMatrix[i][j].screenCoordY) + " " + std::to_string(pointsProjectedWithErrorMatrix[i][j].pictureId) + "\n";

			// fill labels txt
			labelsFileStreamW << std::to_string(labels[i][j]) + "\n";

			// fill labels detailed txt
			labelsDetailedFileStreamW << std::to_string(labelsDetailed[i][j]) + "\n";
		}
	}


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

	//log.close();
}
