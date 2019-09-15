REM silently turns off command echoing, and only output the batch author intended to be written is actually written.
@ECHO OFF

REM always have to use SETLOCAL ENABLEDELAYEDEXPANSION and !...! instead of %...% when working with variables which are modified inside a loop.
setlocal enabledelayedexpansion

REM create the directory that will hold all colored point clouds
if not exist ".\_EveryPointCloud\" ( 
	mkdir .\_EveryPointCloud
)

REM counts the number of directories in the current directory
for /f %%a in ('dir /b /ad %folder%^|find /c /v "" ') do (
	set count=%%a
)
REM create variable which holds the number of directories that hold point cloud data
REM ignores the directory created at the beginning of this program
set /A dirPointCloudCounter = %count%-1

if not exist ".\LiDAR_PointCloud\" (
	dirPointCloudCounter = %count%
)

REM create string variable
set dirPointCloudBaseName=LiDAR_PointCloud

set cloudPointTxtFileName=LiDAR_PointCloud_points.txt
set cloudPointErrorTxtFileName=LiDAR_PointCloud_error.txt
set destDirNameForGeneratedPlyFiles=.\_EveryPointCloud\

set baseCloudPointFileName=LiDAR_PointCloud.ply
set baseErrorCloudPointFileName=LiDAR_PointCloud_error.ply
set baseLabelsFileName=LiDAR_PointCloud_labels

REM Starts at 1, steps by one, and finishes at dirPointCloudCounter.
REM %%x if inside batch file; if in console use %x
for /l %%x in (1, 1, %dirPointCloudCounter%) do ( 
	SET "dirNum=%%x"
    SET dirName=%dirPointCloudBaseName%!dirNum!
    ECHO ---- Processing data in the !dirName! directory
	
	REM if the directory exsits
	if exist ".\!dirName!\" ( 
		echo ".\!dirName!\%cloudPointTxtFileName%"
		
		REM create the colored point clouds
		python colorize.py ".\!dirName!\%cloudPointTxtFileName%"
		python colorize.py ".\!dirName!\%cloudPointErrorTxtFileName%"
		
		REM after using the base point cloud file (LiDAR_PointCloud.ply), delete it
		del ".\!dirName!\%baseCloudPointFileName%" 
		del ".\!dirName!\%baseErrorCloudPointFileName%" 
		del ".\!dirName!\*.bmp" 
		
		cd !dirName!
		ren "%baseLabelsFileName%.txt" "??????????????????????????????????????!dirNum!.*"
		move "%baseLabelsFileName%*" "..\_EveryPointCloud"
		del ".\*.txt" 
		
		REM quotes used for file with spaces in the name
		ren "*.ply" "??????????????????????????????????????!dirNum!.*"
		move "*.ply" "..\_EveryPointCloud"
		cd ..
		
		rmdir .\!dirName!
	)
)

endlocal
