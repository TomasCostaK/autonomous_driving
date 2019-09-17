#
# This script colors point clouds according to the label assigned to each point.
# Supports uncolored and colored point clouds as input.
# Each unique label is assigned a random rgb color before adding it to the appropriate points.
#

import random
import sys


# from an array of repeated values, return a new array which holds all the unique values in it
def getAllUniqueValuesFromList(seq):
	b = []
	for i in seq:
		if i not in b:
			b.append(i)
	return b
	
arg1 = sys.argv[1]	# path to the the point cloud file (.ply) to be colored
arg2 = sys.argv[2]  # path to the text file that holds the label information
arg4 = sys.argv[3]	# 0 means the original point cloud file is uncolored;
					# 1 means the original point cloud file is colored;

# create a name for the generated label colored point cloud file
pointCloudFilenameRecolored = arg1.split('.')[0] + '_recolored' + '.ply'
pointCloudFilename = arg1
labelsFilename = arg2

numLinesInPlyHeader = -1

if int(arg4) == 0:				# the original point cloud is uncolored
	numLinesInPlyHeader = 7
elif int(arg4) == 1:			# the original point cloud is colored
	numLinesInPlyHeader = 10
else:
	print('The 3rd argument has to have a value of:\n\t - 0: if the original point cloud is uncolored\n\t - 1: if the original point cloud is colored')
	sys.exit()

# store every line of the point cloud file in a list of strings (without \n character at the end of each line)
# The first 10 elements correspond to the header of a .ply file
pointCloudLines = [line.rstrip('\n') for line in open(pointCloudFilename)]

plyHeaderLines = []

if numLinesInPlyHeader == 7:
	plyHeaderLines.extend(pointCloudLines[:numLinesInPlyHeader-1]) # do not include the end header for supporting only uncolored point cloud
	plyHeaderLines.extend(['property uchar red'])
	plyHeaderLines.extend(['property uchar green'])
	plyHeaderLines.extend(['property uchar blue'])
	plyHeaderLines.extend(['end_header'])
elif numLinesInPlyHeader == 10:
	plyHeaderLines.extend(pointCloudLines[:numLinesInPlyHeader])


# remove the header elements/lines
pointCloudLines = pointCloudLines[numLinesInPlyHeader:]

print("pointCloudLines length: " + str(len(pointCloudLines)))

# get all labels in a list of integers (each label is a number)
labelNumbersList = [int(line) for line in open(labelsFilename)]
print("labelNumbersList length: " + str(len(labelNumbersList)))
# dictitionary that relates a color to an id
colorIdDictionary = dict()

# list of all different ids in the labelNumbersList
differentIds = getAllUniqueValuesFromList(labelNumbersList)

# assign a color to each id; each color is going to be a string of 3 numbers between 0 and 255
for i in range(0, len(differentIds)):
	
	# generate random rgb color for the id of the current iteration
	r = random.randint(0, 256)   # random value between [0, 255]
	g = random.randint(0, 256)
	b = random.randint(0, 256)
	
	# tuple of rgb
	rgbKeyString = (r, g, b)
	
	# key is the id, value is the rbg string
	# append a new key value pair into the dictionary
	colorIdDictionary.update({differentIds[i] : rgbKeyString})


newPointCloudLines = []
# change the color of every point cloud dot according to the assignments made previously between id and rgb value
for i in range(len(pointCloudLines)):
	
	# [x, y, z, r, g, b]
	lineList = pointCloudLines[i].split()
	
	if i >= len(labelNumbersList):
		break
	
	# get the point id
	pointId = labelNumbersList[i]
	
	rgbTuple = colorIdDictionary.get(pointId)
	
	newPointCloudLines.append(lineList[0] + ' ' + lineList[1] + ' ' + lineList[2] + ' ' + str(rgbTuple[0]) + ' ' + str(rgbTuple[1]) + ' ' + str(rgbTuple[2]))


with open(pointCloudFilenameRecolored, mode="w") as outfile: 
	# header
	for i in range(0, len(plyHeaderLines)):
		line = plyHeaderLines[i] + '\n'
		outfile.write(line) 
	
	# points
	for i in range(0, len(newPointCloudLines)):
		
		line = newPointCloudLines[i] + '\n'
		outfile.write(line)      


