REM turns off command echoing
@echo OFF

REM This command and !...! instead of %...% are used when working with variables that are modified inside a loop
setlocal enabledelayedexpansion

REM Counts the number of directories in the current directory
REM Expects that the current directory only has LiDAR_PointCloudX directories
for /f %%a in ('dir /b /ad %folder%^|find /c /v "" ') do (
	set count=%%a
)

set destDir=.\_EveryPointCloud\
REM Create the directory that will hold all colored point clouds
if not exist "%destDir%" ( 
	mkdir "%destDir%"
)

REM number of directories in the current directory
set dirPointCloudCounter=%count%

if exist ".\LiDAR_PointCloud\" (
	REM Ignore the directory created by the manual LiDAR scan, if it exists
	@set /a "dirPointCloudCounter=%count%-1"
)

REM Template of the name of the directories that hold all the data needed to create the colored point clouds
set dirPointCloudBaseName=LiDAR_PointCloud

REM The following file names are present in every directory holding point cloud data
set cloudPointTxtFileName=LiDAR_PointCloud_points.txt
set cloudPointErrorTxtFileName=LiDAR_PointCloud_error.txt
REM Name of the ideal uncolored point cloud, 
set baseCloudPointFileName=LiDAR_PointCloud.ply
REM Name of the uncolored point cloud with error
set baseErrorCloudPointFileName=LiDAR_PointCloud_error.ply
REM Name of the the file that holds the restrictive labels
set baseLabelsFileName=LiDAR_PointCloud_labels
REM Name of the the file that holds the detailed labels
set baseLabelsDetailedFileName=LiDAR_PointCloud_labelsDetailed

REM Starts at 1, steps by one, and finishes at dirPointCloudCounter.
REM use %%x if inside batch file; use %x if in the console
for /l %%x in (1, 1, %dirPointCloudCounter%) do ( 
	set "dirNum=%%x"
    set dirName=%dirPointCloudBaseName%!dirNum!
	
    echo ---- Processing data in the !dirName! directory
	
	REM if the directory exsits
	if exist ".\!dirName!\" ( 
		REM create the colored point clouds
		python colorize.py ".\!dirName!\%cloudPointTxtFileName%"
		python colorize.py ".\!dirName!\%cloudPointErrorTxtFileName%"
		
		REM after using the base point cloud files (LiDAR_PointCloud.ply and LiDAR_PointCloud_error.ply), delete them
		del ".\!dirName!\%baseCloudPointFileName%" 
		del ".\!dirName!\%baseErrorCloudPointFileName%" 
		REM delete the pictures
		del ".\!dirName!\*.bmp" 
		
		REM Rename the file holding the restricted labels by adding an index at the end
		REM And move the file to the destination directory
		cd !dirName!
		ren "%baseLabelsFileName%.txt" "??????????????????????????????????????!dirNum!.*"
		move "%baseLabelsFileName%!dirNum!.txt" ".%destDir%"
		
		REM Rename the file holding the detailed labels by adding an index at the end 
		REM And move the file to the destination directory
		ren "LiDAR_PointCloud_labelsDetailed.txt" "??????????????????????????????????????????????????????????!dirNum!.*"
		move "%baseLabelsDetailedFileName%!dirNum!.txt" ".%destDir%"
		
		del ".\*.txt" 
		
		REM Rename all the colored point clouds to have an index at the end of their name
		REM Move all of them to the destination directory
		ren "*.ply" "??????????????????????????????????????!dirNum!.*"
		move "*.ply" ".%destDir%"
		cd ..
		
		REM remove the directory that was processed in the current iteration
		rmdir .\!dirName!
	)
)

endlocal
