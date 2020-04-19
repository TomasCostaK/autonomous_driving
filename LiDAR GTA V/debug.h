#pragma once

#include "inc/types.h"

void logString(std::string filePath, std::string str);

// player logs
void LogPlayerWorldPos(std::string filePath, std::string functionName);
void LogPlayerWorldRot(std::string filePath, std::string functionName);
void LogPlayerForwardDirection(std::string filePath, std::string functionName);
void LogPlayerAlphaValue(std::string filePath, std::string functionName);

// lidar logs
void LogLidarHeight(std::string filePath, std::string functionName, Vector3 centerDot);

// camera logs
void LogRenderingCameraSettings(std::string filePath, std::string functionName, Cam camera);
void LogGameplayCameraFov(std::string filePath, std::string functionName);
void LogCameraRotation(std::string filePath, std::string functionName, Cam camera);
void LogCameraForwardDirection(std::string filePath, std::string functionName, Cam camera);

// point cloud logs
void LogMaxNumberOfPoints(std::string filePath, std::string functionName, float horiFovMin, float horiFovMax, float horiStep, float vertFovMin, float vertFovMax, float vertStep);

// vehicle logs
//std::vector<Vector3> LogVehicleInformation(std::string filePath, std::string functionName, Entity vehicleHandle, std::string entityType, int resolutionX, int resolutionY, Vector3 centerDot);
void LogVehicleCornersCoords(std::string filePath, std::string functionName, Entity vehicleHandle, float vehicleAngleZ, std::vector<Vector3> corners, Vector3 centerDot, Vector3 vehiclePos, int resolutionX, int resolutionY, bool cornersRotated);
void LogVehicleRotations(std::string filePath, std::string functionName, Entity vehicleHandle);

// custom log
void LogCustomInformation(std::string filePath, std::string functionName, std::string title, std::string dataToAppend);






Vector3 transformPointWorldToLocal(std::vector<std::vector<float>> transformMatrix, Vector3 point);

//std::vector<Vector3> GetBestMinMaxCoords(std::string filePath, std::vector<Vector3> corners, Vector3 vehiclePos, int resolutionX, int resolutionY);



