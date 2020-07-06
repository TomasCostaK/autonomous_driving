from GtaView import GtaView
import os.path

# path to directory only containng image files
imagesDir = "/home/jota/Documents/UA/TAA/images_t2/"

# dont use the same directory as the input!
outputpath = '/home/jota/Documents/UA/TAA/logs_t2/v11/'

for subdir, dirs, files in os.walk(imagesDir):
    for filename in files:
        view = GtaView(imagesDir, filename)

        view.transformImageForKittiDataset(kittiImageOutputDir = outputpath)






