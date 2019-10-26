import numpy as np
from scipy import misc
from depth_to_point_cloud import depth_to_point_cloud
from point_cloud_to_image import trim_to_roi
from PIL import Image
import tqdm
import glob
import shutil
import os
import sys
import imageio

ROI = 80        # Region of interest side length, in meters.
CELLS = 600     # Number of cells on a side in the output image
FAR = 1000      # Far plane distance
THRESHOLD = 1.5
name = 0


#ALTITUDE_RES = 0.16
#HORIZONTAL_RES = 1.33  # Velodyne 32 has 0.1,0.2,0.3 and 0.4 

vertical = [-25,-1,-1.667,-15.639,-11.31,0,-0.667,-8.843,-7.254,0.333,-0.333,-6.148,-5.333,
            1.333,0.667,-4,-4.667,1.667,1,-3.667,-3.333,3.333,2.333,-2.667 ,-3,7,4.667,-2.333 ,-2,15,10.333,-1.333] 
vertical.sort()

ALTITUDE_RES = vertical # float(sys.argv[1])
HORIZONTAL_RES = 0.2 # float(sys.argv[2])
### RPM HORIZONTAL_RES
#   300    0.1°
#   600    0.2°
#   900    0.3°
#  1200    0.4°

# To remove previous files and prevent overlap
files = [glob.glob('../convert_image/Pointcloud_images/*'),glob.glob('../convert_image/Pointclouds/*'),glob.glob('../convert_image/Labels/*'),glob.glob('../convert_image/head/*'),glob.glob('../convert_image/left/*'),glob.glob('../convert_image/tail/*'),glob.glob('../convert_image/right/*'),glob.glob('../convert_image/lidar/*')]
for i in range(len(files)):
    for f in files[i]:
        os.remove(f)

# Copy datacollected_images to convert_image for processing
images = [glob.glob('../datacollected_[i]*/head/*'),glob.glob('../datacollected_[i]*/left/*'),glob.glob('../datacollected_[i]*/tail/*'),glob.glob('../datacollected_[i]*/right/*'),glob.glob('../datacollected_[i]*/lidar/*')]
angles = ['head/','left/','tail/','right/','lidar/']
for i in range(len(images)):
    for f in images[i]:
        shutil.copy(f,'../convert_image/'+angles[i])    

heads = glob.glob("../convert_image/head/*")
tails = glob.glob("../convert_image/tail/*")
lefts = glob.glob("../convert_image/left/*")
rights = glob.glob("../convert_image/right/*")

for head, tail, left, right in zip(heads, tails, lefts, rights):

	name = head[len("../convert_image/head/"):-4]

	head = imageio.imread(head)
	tail = imageio.imread(tail)
	left = imageio.imread(left)
	right = imageio.imread(right)

	# Convert depth maps to 3D point cloud
	point_cloud = depth_to_point_cloud(head, tail, left, right, FAR,ALTITUDE_RES,HORIZONTAL_RES,interpolate=True, threshold=THRESHOLD)

	# Trim point cloud to only contain points within the region of interest
	point_cloud = trim_to_roi(point_cloud,ROI)
	# TODO Count number of points within each grid cell
	# TODO Save as image

	grid = np.zeros([CELLS,CELLS])

	for point in point_cloud:
	    x, y, z = point
	    x += ROI/2
	    x /= ROI
	    cell_x = int(x*CELLS)

	    y += ROI/2
	    y /= ROI
	    cell_y = int(y*CELLS)

	    grid[cell_x,cell_y] += 1

	grid = 64*grid

	img = Image.fromarray(grid)
	img = img.rotate(180)
	img.convert('RGB').save("../convert_image/Pointcloud_images/{}.png".format(name))
	np.savetxt("../convert_image/Pointclouds/{}.bin".format(name),point_cloud.tolist())

# Copying labels for processing 
for label in glob.glob('../datacollected_[l]*/*'):
    shutil.copy(label,'../convert_image/Labels/')
