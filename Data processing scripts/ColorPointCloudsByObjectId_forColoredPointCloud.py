import random

def getAllUniqueValuesFromList(seq):
	b = []
	for i in seq:
		if i not in b:
			b.append(i)
	return b

pointCloudFilenameRecolored = 'LiDAR_PointCloud_points_PC_Day1_Recolored.ply'
pointCloudFilename = 'LiDAR_PointCloud_points_PC_Day1.ply'
numLinesInPlyHeader = 10 
labelsFilename = 'LiDAR_PointCloud_labels1.txt'

# store every line in the point cloud file in a list o strings (without 7n character at the end of each line)
# The first 10 elements correspond to the header of a .ply file
pointCloudLines = [line.rstrip('\n') for line in open(pointCloudFilename)]

plyHeaderLines = pointCloudLines[:numLinesInPlyHeader]
print(plyHeaderLines)
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
	#print(pointCloudLines[i])


with open(pointCloudFilenameRecolored, mode="w") as outfile:  # also, tried mode="rb"
	# header
	for i in range(0, len(plyHeaderLines)):
		line = plyHeaderLines[i] + '\n'
		outfile.write(line) 
	
	# points
	for i in range(0, len(newPointCloudLines)):
		line = newPointCloudLines[i] + '\n'
		outfile.write(line)      


