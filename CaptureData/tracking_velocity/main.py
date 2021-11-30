import cv2 as cv
import csv
import math
import numpy as np
import scipy.signal
from matplotlib import pyplot as plt

agentLength = 23.85 	# cm
fourLegoLength = 11.15	# cm
threeLegoLength = 7.9	# cm


video = 2

if video == 0:
	fileName = 'diego.mp4'
elif video == 1:
	fileName = 'felipe.mp4'
elif video == 2:
	fileName = 'cristobal.mp4'
else:
	fileName = 'mopa.mp4'

#fileName = 'control_no_filter.MOV'

vid = cv.VideoCapture(fileName)
if not vid.isOpened():
	exit(-1)
fps = vid.get(cv.CAP_PROP_FPS)
deltaT = 1./fps

ret, frame = vid.read()

# get agent length in pixels
bbox = cv.selectROI('Tracking_v2', frame, False)
length = bbox[2]

# initialise tracker
bbox = cv.selectROI('Tracking_v2', frame, False)
tracker = cv.TrackerCSRT_create()
tracker.init(frame, bbox)

# write CSV file
myFile = open('plot.csv', 'w', newline = '')
writer = csv.writer(myFile)
writer.writerow(['time(s)', 'distance(cm)', 'velocity(cm/s)'])

firstFrame = True
seconds = 0.
firstPoint = (0,0)
x, y, w, h = 0, 0, 0, 0
previous = 0.
deltaX = 0.
velocity = 0.
timeArr = []
velocityArr = []
distanceArr = []

while vid.isOpened():
	ret, frame = vid.read()
	if ret:
		ret, bbox = tracker.update(frame)
		if ret:
			x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
			cv.rectangle(frame, (x,y), (x+w, y+h), (255, 0, 0), 2, 1)
		else:
			print('could not detect object')
			exit(-1)

		p = (x,y)
		if firstFrame:
			firstPoint = p

		# print [distance traveled (cm), time elapsed (s)]

		distance = math.sqrt((p[0] - firstPoint[0])**2 + (p[1] - firstPoint[1])**2) * agentLength / length
		deltaX = distance - previous
		velocity = deltaX/deltaT

		# write CSV file
		writer.writerow([seconds, distance, velocity])

		# append into arrays
		timeArr = np.append(timeArr, seconds)
		velocityArr = np.append(velocityArr, velocity)
		distanceArr = np.append(distanceArr, distance)

		cv.imshow('Tracking_v2', frame)

		seconds += deltaT
		firstFrame = False
		previous = distance

		if cv.waitKey(1) == 27:
			break
	else:
		break
vid.release()

# median filter
velocityArr = scipy.signal.medfilt(velocityArr, 11)

# butterworth filter
b, a = scipy.signal.butter(1, 0.5)
velocityArr = scipy.signal.filtfilt(b, a, velocityArr)

plt.figure()
plt.plot(timeArr, velocityArr)
plt.ylim(-8, 8)
plt.xlabel('time(s)')
plt.ylabel('velocity(cm/s)')
#plt.title(fileName)
plt.grid(True)
plt.show()