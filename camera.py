import cv2
import time
from threading import Thread, Lock

from logo_recognition import detect_logo

#Camera-related functions, including image recognition

shutdown_signal = False
image = None
image_lock = Lock()

camera_thread = None

#Function that handles the camera in the background (DO NOT RUN DIRECTLY)
def handler(cam_id=0, output_file=None):
	global shutdown_signal, image, image_lock

	capture = cv2.VideoCapture(0)
	width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
	height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
	writer = None

	if output_file is not None:
		writer = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc(*'DIVX'), 20, (width, height))

	while True:
		#Get the image from the camera
		success, img = capture.read()
		if output_file is not None: writer.write(img) #If instructed, write to video file

		#Write to the global image var
		if image_lock.acquire(0):
			image = img
			image_lock.release()

		#If instructed, shut down
		if shutdown_signal:
			break
		
	capture.release()
	if writer is not None: writer.release()
	return 0

#Start collecting camera footage (ALWAYS RUN THIS FIRST)
def initialize(cam_id=0, output_file=None):
	global image_lock, camera_thread

	camera_thread = Thread(target=handler, args=(cam_id, output_file))
	camera_thread.start()

	return 0

#Deinitialize the camera handler
def shutdown():
	global shutdown_signal, camera_thread

	shutdown_signal = True
	camera_thread.join()

	return 0

#Get the last image captured by the camera
def get_image():
	global image, image_lock

	img = None
	with image_lock:
		img = image

	return img

#Get the horizontal distance vector from the drone to the logo. Returns a tuple in the form (distance, angle).
#Returns None if the logo wasn't found.
def locate_logo_from_image(img, attitude, altitude, fov):

	SCALE = 0.25
	best_is_white = lambda x: np.min(x >= [0.8, 0.8, 0.8], axis=-1)
	best_is_red = lambda x: (x[:,:,2] > x[:,:,1]*1.5) * (x[:,:,2] > x[:,:,0]*1.2) * x[:,:,2] > 0.7 # In BGR format

	#Locate the logo within the image
	img = cv2.resize(img, (int(img.shape[1] * SCALE), int(img.shape[0] * SCALE)))

	bbox = detect_logo(img,
		is_white=best_is_white,
		is_red=best_is_red,
		expected_sizes=[200, 300],
		granularity=0.2,
		top_regions=5,
		fine_tuning_passes=5)

	#If we didn't find it, return None
	if bbox is None: return None #Double check with Harvard

	#Calculate the distance vector for the logo's location
	center = ((bbox[0] + bbox[2]/2) - img.shape[1]/2, (bbox[1] + bbox[3]/2) - img.shape[0]/2)
	theta = atan2(center[1], center[0]) - attitude.yaw
	r = sqrt(center[0]**2+center[1]**2)

	target = (altitude * tan(attitude.roll + r*cos(theta)), altitude * tan(attitude.pitch + r*sin(theta)))
	
	return target

	
