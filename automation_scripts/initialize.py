
import os
import subprocess
import shutil

# The sdn_kitti_object_trainval.pth file needs to be in the root dir

if not os.path.exists("pseudo_lidar_V2"):
    print("Cloning upstream repo...")
    command = "git clone https://github.com/mileyan/pseudo_lidar_V2"
    result = subprocess.run(command.split(' '), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    #Print the stdout and stderr
    print(result.stdout)
    print(result.stderr)
    print("Done.")

print("Seting up config file...")
os.makedirs("pseudo_lidar_V2/results/sdn_kitti_train_set/", exist_ok=True)
if os.path.exists("sdn_kitti_object_trainval.pth"):
    shutil.move("sdn_kitti_object_trainval.pth", "pseudo_lidar_V2/results/sdn_kitti_train_set/sdn_kitti_object_trainval.pth")
else:
    print("File not found.")
print("Done.")

os.chdir("pseudo_lidar_V2")

print("Creating data dirs...")
subdirs = ["kitti/training", "kitti/testing"]
subsubdirs = ["calib", "image_2", "image_3"]
for i in subdirs:
    for ii in subsubdirs:
        path = os.path.join(i, ii)
        os.makedirs(path, exist_ok=True)
print("Done.")

print("Installing dependencies...")
command = "pip install -r requirements.txt"
result = subprocess.run(command.split(' '), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#Print the stdout and stderr
print(result.stdout)
print(result.stderr)
print("Done.")
