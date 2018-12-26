import matplotlib.pyplot as plt
import numpy as np
import matplotlib.path as mpath
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
import time
import random


def label(xy, text):
    y = xy[1] - 0.15  # shift y-value for label so that it's below the artist
    plt.text(xy[0], y, text, ha="center", family='sans-serif', size=14)

def obj_centroid(size, pos):
	x_sum = 0
	y_sum = 0
	for i in range(size):
		x_sum += pos[i][0]
		y_sum += pos[i][1]

	return((x_sum/size,y_sum/size))

def obj_height(size, pos):
	smallest = 0
	largest = 0
	for i in range(size):
		if pos[i][1] > largest:
			largest = pos[i][1]
		if pos[i][1] < smallest:
			smallest = pos[i][1]
	return largest-smallest

def obj_width(size, pos):
	smallest = 0
	largest = 0
	for i in range(size):
		if pos[i][0] > largest:
			largest = pos[i][0]
		if pos[i][0] < smallest:
			smallest = pos[i][0]
	return largest-smallest

def create_figure(size,pos):
	ob_cen = obj_centroid(size,pos)
	ob_hei = obj_height(size,pos)
	ob_wid = obj_width(size, pos)

	return mpatches.Rectangle(ob_cen, ob_wid, ob_hei,fill = False)

# plt.ion()


pos = ((1,1), (2,1), (4,5),(1,7), (5,12), (3,9),(4,1), (2,10), (6,6),(3,3), (2,2), (7,7))
pos1 = ((80,76), (22,100), (56,77),(10,79), (58,99), (43,89),(49,100), (62,90), (60,60),(77,91), (0,82), (76,76))

print(obj_centroid(len(pos),pos))

fig, ax = plt.subplots()
patches = []


# add a rectangle
r = create_figure(len(pos), pos)
r50 = create_figure(len(pos1), pos1)
patches.append(r)
patches.append(r50)


a = {}
for i in range(40):
	xsample = random.randint(-100, 300)
	ysample = random.randint(-100, 300)
	key = "r"+str(i)
	value = create_figure(len(pos),pos)

	a[key] = value
	patches.append(a[key])
	patches[i].set_x(xsample)
	patches[i].set_y(ysample)

print(r)
print(a["r0"])
print(a)

colors = np.linspace(0, 1, len(patches))
collection = PatchCollection(patches, cmap=plt.cm.hsv, alpha=0.3)
collection.set_array(np.array(colors))
ax.add_collection(collection)

print(ax)
# print(ax[0])
print(collection.get_array())

plt.ion()
# plt.axis('equal')
plt.axis('off')
# plt.tight_layout()
# plt.plot(1300,5999)
# plt.plot(1000,1000)
ax.set_xlim(-100, 300)
ax.set_ylim(-100, 300)
plt.show()





for i in range(20):

	xsample = random.randint(-100, 300)
	ysample = random.randint(-100, 300)
	chooseRec = random.randint(0, 39)
	patches[chooseRec].set_x(xsample)
	patches[chooseRec].set_y(ysample)
	print(i,chooseRec, patches[chooseRec].get_x(),patches[chooseRec].get_y())

	colors = np.linspace(0, 1, len(patches))
	collection = PatchCollection(patches, cmap=plt.cm.hsv, alpha=0.3)
	collection.set_array(np.array(colors))
	ax.add_collection(collection)
	plt.pause(1e-17)
	time.sleep(0.1)
	plt.cla()
	plt.axis('off')
	ax.set_xlim(-100, 300)
	ax.set_ylim(-100, 300)
	plt.show()


plt.show()