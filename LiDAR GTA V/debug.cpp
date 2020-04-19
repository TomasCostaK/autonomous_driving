#include <string>
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
#include "inc\types.h"
#include "inc\natives.h"
#include "script.h"

std::ofstream logFileW;

void logString(std::string filePath, std::string str)
{
	// append
	logFileW.open(filePath, std::ios_base::app);
	logFileW << str;
	logFileW.close();
}

void LogGameplayCameraFov(std::string filePath, std::string functionName)
{
	std::string line = "[ " + functionName + "() ] Gameplay camera FOV: ";

	line += std::to_string(CAM::GET_GAMEPLAY_CAM_FOV());

	logString(filePath, line + "\n");
}

/*
	Log character position in world space coordinates.
	The position correspondents roughly to the middle of the character model.
*/
void LogPlayerWorldPos(std::string filePath, std::string functionName)
{
	std::string line = "[ " + functionName + "() ] Character position in world space: ";

	// get player position (middle of the character model)
	Vector3 playerCurrentPos = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(PLAYER::PLAYER_PED_ID(), 0, 0, 0);

	logString(filePath, line + std::to_string(playerCurrentPos.x) + " " + std::to_string(playerCurrentPos.y) + " " + std::to_string(playerCurrentPos.z) + "\n");
}

void LogPlayerForwardDirection(std::string filePath, std::string functionName)
{
	std::string line = "[ " + functionName + "() ] Character forward (local y axis) direction: ";

	Vector3 playerForwardDirection = ENTITY::GET_ENTITY_FORWARD_VECTOR(PLAYER::PLAYER_PED_ID());

	logString(filePath, line + std::to_string(playerForwardDirection.x) + " " + std::to_string(playerForwardDirection.y) + " " + std::to_string(playerForwardDirection.z) + "\n");
}

void LogCameraForwardDirection(std::string filePath, std::string functionName, Cam camera)
{
	std::string line = "[ " + functionName + "() ] Camera forward (local y axis) direction: ";

	Vector3 camRot = CAM::GET_CAM_ROT(camera, 1);

	Vector3 worldForwardVector; worldForwardVector.x = 0; worldForwardVector.y = 1; worldForwardVector.z = 0;

	Vector3 camForwardVector = rotate_point_around_z_axis(worldForwardVector, camRot.z);

	line += std::to_string(camForwardVector.x) + " " + std::to_string(camForwardVector.y) + " " + std::to_string(camForwardVector.z);

	logString(filePath, line + "\n");
}

void LogCameraRotation(std::string filePath, std::string functionName, Cam camera)
{
	std::string line = "[ " + functionName + "() ] Camera rotation: ";

	Vector3 camRot = CAM::GET_CAM_ROT(camera, 1);

	line += std::to_string(camRot.x) + " " + std::to_string(camRot.y) + " " + std::to_string(camRot.z);

	logString(filePath, line + "\n");
}

void LogCustomInformation(std::string filePath, std::string functionName, std::string title, std::string dataToAppend)
{
	std::string line = "[ " + functionName + "() ] " + title + ": ";

	line += dataToAppend;

	logString(filePath, line + "\n");
}

/*
	Log character rotation in world space coordinates.
*/
void LogPlayerWorldRot(std::string filePath, std::string functionName)
{
	std::string line = "[ " + functionName + "() ] Character rotation in world space: ";

	Vector3 rot = ENTITY::GET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), 0);

	logString(filePath, line + std::to_string(rot.x) + " " + std::to_string(rot.y) + " " + std::to_string(rot.z) + "\n");
}

/*
	Log character rotation in world space coordinates.
*/
void LogVehicleRotations(std::string filePath, std::string functionName, Entity vehicleHandle)
{
	std::string line = "[ " + functionName + "() ] Vehicle rotation experiment: ";

	Vector3 camRot = ENTITY::GET_ENTITY_ROTATION(PLAYER::PLAYER_PED_ID(), 0);
	Vector3 camForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(PLAYER::PLAYER_PED_ID());

	line += "\n\Entity: " + std::to_string(vehicleHandle);

	line += "\n\tCamera rotation: " + std::to_string(camRot.x) + " " + std::to_string(camRot.y) + " " + std::to_string(camRot.z);
	line += "\n\tCamera Forward vector (points in -Y direction): " + std::to_string(camForwardVector.x) + " " + std::to_string(camForwardVector.y) + " " + std::to_string(camForwardVector.z);

	Vector3 vehicleRot = ENTITY::GET_ENTITY_ROTATION(vehicleHandle, 1);
	Vector3 vehicleForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(vehicleHandle);	 // local y vector

	line += "\n\tVehicle rotation: " + std::to_string(vehicleRot.x) + " " + std::to_string(vehicleRot.y) + " " + std::to_string(vehicleRot.z);
	line += "\n\tVehicle Forward vector (points in Y direction): " + std::to_string(vehicleForwardVector.x) + " " + std::to_string(vehicleForwardVector.y) + " " + std::to_string(vehicleForwardVector.z);

	logString(filePath, line + "\n");
}

/*
	Log distance from the Lidar and camera to ground.
	Both sensors are positioned at "centerDot"
*/
void LogLidarHeight(std::string filePath, std::string functionName, Vector3 centerDot)
{
	std::string line = "[ " + functionName + "() ] Lidar and Camera position in world space: ";

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
	float distanceToGround = distanceBetween3dPoints(centerDot, collisionPoint);

	logString(filePath, line + std::to_string(distanceToGround) + "\n");
}

/*
	Log character transparency value.
*/
void LogPlayerAlphaValue(std::string filePath, std::string functionName)
{
	std::string line = "[ " + functionName + "() ] Character alpha value: ";

	int alpha = ENTITY::GET_ENTITY_ALPHA(PLAYER::PLAYER_PED_ID());

	logString(filePath, line + std::to_string(alpha) + "\n");
}

/*
	Log the settings of the camera used to create the image views.
*/
void LogRenderingCameraSettings(std::string filePath, std::string functionName, Cam camera)
{
	std::string line = "[ " + functionName + "() ] Rendering camera settings: ";

	Vector3 camCoords = CAM::GET_CAM_COORD(camera);
	Vector3 camRot = CAM::GET_CAM_ROT(camera, 1);
	float camFov = CAM::GET_CAM_FOV(camera);
	float camNearClip = CAM::GET_CAM_NEAR_CLIP(camera);
	float camFarClip = CAM::GET_CAM_FAR_CLIP(camera);
	float camDof = CAM::GET_CAM_FAR_DOF(camera);

	std::string strToWrite = "\n\tIndex: " + std::to_string(camera) + "\n\tCamera position: " + std::to_string(camCoords.x) + ", " + std::to_string(camCoords.y) + ", " + std::to_string(camCoords.z) + "\n\tCamera Rotation: " +
		std::to_string(camRot.x) + ", " + std::to_string(camRot.y) + ", " + std::to_string(camRot.z) + "\n\tCamera FOV: " +
		std::to_string(camFov) + "\n\tCamera Near Clip Field: " + std::to_string(camNearClip) + "\n\tCamera Far Clip Field: " + std::to_string(camFarClip) + "\n\tCamera Depth of Field: " + std::to_string(camDof);

	logString(filePath, line + strToWrite + "\n");
}

/*
	Log the maximum number of points that a point cloud frame can have.
*/
void LogMaxNumberOfPoints(std::string filePath, std::string functionName, float horiFovMin, float horiFovMax, float horiStep, float vertFovMin, float vertFovMax, float vertStep)
{
	std::string line = "[ " + functionName + "() ] Max number of points per frame: ";

	int vertexCount = (horiFovMax - horiFovMin) * (1 / horiStep) * (vertFovMax - vertFovMin) * (1 / vertStep);		// theoretical vertex count (if all raycasts intercept with an object)

	logString(filePath, line + std::to_string(vertexCount) + "\n");
}

/*
	Logs a vehicle box corner positions in the local coordinate system of the vehicle, world coordinate system, camera coordinate system, projection onto image view
*/
void LogVehicleCornersCoords(std::string filePath, std::string functionName, Entity vehicleHandle, float vehicleAngleZ, std::vector<Vector3> corners, Vector3 centerDot, Vector3 vehiclePos, int resolutionX, int resolutionY, bool cornersRotated)
{
	std::string line = "[ " + functionName + "() ] Vehicle Entity " + std::to_string(vehicleHandle) + " information:";

	if (!cornersRotated)
	{
		line += "\n\tVehicle box corners in the local coordinate system, without any rotation: ";
		// print corners position in the vehicle coordinate system, without any rotation applied to them
		for (int i = 0; i < 8; i++)
		{
			line += "\n\t\tcorners[" + std::to_string(i) + "]: " + std::to_string(corners[i].x) + ", " + std::to_string(corners[i].y) + ", " + std::to_string(corners[i].z);
		}

		line += "\n\tVehicle box corners in the local coordinate system, with " + std::to_string(vehicleAngleZ) + "º rotation around local (vehicle) Z axis";
		// rotate vehicle box corners according to the vehicle's rotation around z axis
		for (int i = 0; i < 8; i++)
		{
			// rotation around z axis
			corners[i] = rotate_point_around_z_axis(corners[i], vehicleAngleZ);
			line += "\n\t\tcorners[" + std::to_string(i) + "]: " + std::to_string(corners[i].x) + ", " + std::to_string(corners[i].y) + ", " + std::to_string(corners[i].z);
		}
	}
	else
	{
		line += "\n\tVehicle box corners in the local coordinate system, with " + std::to_string(vehicleAngleZ) + "º rotation around local (vehicle) Z axis";
		for (int i = 0; i < 8; i++)
		{
			line += "\n\t\tcorners[" + std::to_string(i) + "]: " + std::to_string(corners[i].x) + ", " + std::to_string(corners[i].y) + ", " + std::to_string(corners[i].z);
		}
	}

	line += "\n\tVehicle box corners in the camera coordinate system, with " + std::to_string(vehicleAngleZ) + "º rotation around local (vehicle) Z axis";
	for (int i = 0; i < 8; i++)
	{
		line += "\n\t\tcorners[" + std::to_string(i) + "]: " + std::to_string(corners[i].x + vehiclePos.x - centerDot.x) + ", " + std::to_string(corners[i].y + vehiclePos.y - centerDot.y) + ", " + std::to_string(corners[i].z + vehiclePos.z - centerDot.z);
	}

	line += "\n\tVehicle box corners in the world coordinate system, with " + std::to_string(vehicleAngleZ) + "º rotation around local (vehicle) Z axis";
	for (int i = 0; i < 8; i++)
	{
		line += "\n\t\tcorners[" + std::to_string(i) + "]: " + std::to_string(corners[i].x + vehiclePos.x) + ", " + std::to_string(corners[i].y + vehiclePos.y) + ", " + std::to_string(corners[i].z + vehiclePos.z);
	}

	line += "\n\tVehicle projected corners coordinates:";
	for (int i = 0; i < 8; i++)
	{
		float projectedX;
		float projectedY;
		GRAPHICS::_WORLD3D_TO_SCREEN2D(corners[i].x + vehiclePos.x, corners[i].y + vehiclePos.y, corners[i].z + vehiclePos.z, &projectedX, &projectedY);
		
		line += "\n\t\tcorners[" + std::to_string(i) + "]: " + std::to_string((int)(projectedX * resolutionX * 1.5)) + ", " + std::to_string((int)(projectedY * resolutionY * 1.5));
	}

	logString(filePath, line + "\n");
}

std::vector<std::vector<float>> worldToLocalTransformMatrix_3x3(Vector3 globalXaxis, Vector3 globalYaxis, Vector3 globalZaxis, Vector3 localXaxis, Vector3 localYaxis, Vector3 localZaxis)
{
	std::vector<std::vector<float>> matrix = std::vector<std::vector<float>>(3, std::vector<float>(3));

	std::vector<Vector3> globalAxis = { globalXaxis, globalYaxis, globalZaxis };

	for (int i = 0; i < 3; i++)
	{
		matrix[i][0] = dotProduct3D(globalAxis[i], localXaxis);
		matrix[i][1] = dotProduct3D(globalAxis[i], localYaxis);
		matrix[i][2] = dotProduct3D(globalAxis[i], localZaxis);
	}

	return matrix;
}

Vector3 transformPointWorldToLocal(std::vector<std::vector<float>> transformMatrix, Vector3 point)
{
	Vector3 newPoint;

	newPoint.x = transformMatrix[0][0] * point.x + transformMatrix[0][1] * point.y + transformMatrix[0][2] * point.z;
	newPoint.y = transformMatrix[1][0] * point.x + transformMatrix[1][1] * point.y + transformMatrix[1][2] * point.z;
	newPoint.z = transformMatrix[2][0] * point.x + transformMatrix[2][1] * point.y + transformMatrix[2][2] * point.z;

	return newPoint;
}





