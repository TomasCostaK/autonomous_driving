from GtaView import GtaView
import os.path

# path to directory only containng image files
imagesDir = "D:/diogo/Desktop/qqqq/"

# dont use the same directory as the input!
outputpath = 'D:/diogo/Desktop/KittiSizedImages/'

for subdir, dirs, files in os.walk(imagesDir):
    for filename in files:
        view = GtaView(imagesDir, filename)

        view.transformImageForKittiDataset(kittiImageOutputDir = outputpath)






