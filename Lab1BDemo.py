import time
import numpy as np
import math
import heapq

from Motor import *            
PWM=Motor() 

from Ultrasonic import *
ultrasonic=Ultrasonic() 

from servo import *
sight_servo =Servo()

from picamera2 import Picamera2
picam = Picamera2()

def scan(angle):
	sight_servo.setServoPwm('0',angle)
	return ultrasonic.get_distance()


def insert_with_blast_radius(map_, map_size, x_distance, y_distance, radius):
	# 7 squares on my desk = 80 cm measurement, so 1 cell in the map represents ~11.5 cm
	relative_x_distance, relative_y_distance = int(x_distance / 3), int(y_distance / 3 + 2)
	
	x, y = (map_size // 2) + relative_x_distance, map_size - relative_y_distance
	print(f"Inserting at {y}, {x}")
	print("")
	if radius == 0 and 0 < x < map_size and 0 < y < map_size:
		map_[y, x] = 1
	for xr in range(-radius, radius):
		if 0 < y < map_size and 0 < x + xr < map_size:
			map_[y, x + xr] = 1
	return map_
	
	
def update_map(map_, map_size, angle, distance, blast_radius):
	if distance == 0: # distance of 0 cm away means nothing was seen
		return map_
	print(f"Distance of object: {distance}")
	angle -= 90 # making straight ahead 0 degrees
	print(f"Converted angle {angle}")
	x_distance = round(distance * math.sin(math.radians(angle)))
	y_distance = round(distance * math.cos(math.radians(angle)))
	print(f"y distance {y_distance} and x distance {x_distance}")
	map_ = insert_with_blast_radius(map_, map_size, x_distance, y_distance, blast_radius)
	return map_


def build_map(map_size, blast_radius):
	map_ = np.zeros((map_size, map_size))
	for eye_angle in range(30, 150):
		distance_of_object = scan(eye_angle)
		map_ = update_map(map_, map_size, eye_angle, distance_of_object, blast_radius)
	print("Calculated Map")
	print(map_)
	return map_
	

def find_polars(map_, map_size, destination):
	d_y, d_x = (map_size - destination[0] - 1), (destination[1] - map_size // 2)
	angle = math.atan(d_x / d_y)
	distance = math.sqrt(d_y**2 + d_x**2)
	return angle * 200, distance
	

def turn_left(angle):
	PWM.setMotorModel(-500, -500, 2000, 2000)  # Left
	time.sleep(angle / 90)
	PWM.setMotorModel(0, 0, 0, 0)  # Stop


def turn_right(angle):
	PWM.setMotorModel(2000, 2000, -500, -500)  # Right
	time.sleep(angle / 90)
	PWM.setMotorModel(0, 0, 0, 0)  # Stop
	

def go_forward(distance):
	PWM.setMotorModel(1000, 1000, 1000, 1000)  # Forward
	time.sleep(distance / 10)
	PWM.setMotorModel(0, 0, 0, 0)  # Stop
	

def go_with_polars(angle, distance):
	if angle < 0:
		turn_left(abs(angle))
	else:
		turn_right(abs(angle))
	go_forward(distance)


def heuristic(a, b):
	return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def astar(array, start, goal):
	neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
	close_set = set()
	came_from = {}
	gscore = {start:0}
	fscore = {start:heuristic(start, goal)}
	oheap = []
	heapq.heappush(oheap, (fscore[start], start))
	while oheap:
		current = heapq.heappop(oheap)[1]
		if current == goal:
			data = []
			while current in came_from:
				data.append(current)
				current = came_from[current]
			route = data
			route = route[::-1]
			return route
		close_set.add(current)
		for i, j in neighbors:
			neighbor = current[0] + i, current[1] + j
			tentative_g_score = gscore[current] + heuristic(current, neighbor)
			if 0 <= neighbor[0] < array.shape[0]:
				if 0 <= neighbor[1] < array.shape[1]:
					if array[neighbor[0]][neighbor[1]] == 1:
						continue
				else:
					# array bound y walls
					continue
			else:
				# array bound x walls
				continue
			if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
				continue
			if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
				came_from[neighbor] = current
				gscore[neighbor] = tentative_g_score
				fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
				heapq.heappush(oheap, (fscore[neighbor], neighbor))
	return False


def find_path(map_, map_size, goal):
	return astar(map_, (map_size - 1, map_size // 2), goal)
		

def go_based_on_map(map_, map_size, goal, steps):
	path = find_path(map_, map_size, goal)
	destination = path[steps]
	print(f"Calculated destination: {destination}")
	angle, distance = find_polars(map_, map_size, destination)
	print(f"Calculated angle and distance: {(angle, distance)}")
	go_with_polars(angle, distance)
	

def red_seen(angle):
	sight_servo.setServoPwm('0',angle)
	picam.start()
	time.sleep(1)
	array = picam.capture_array("main")
	picam.stop()
	r, g, b = np.mean(array[:,:,0]), np.mean(array[:,:,1]), np.mean(array[:,:,2])
	return r > g and r > b
	
	
def is_red_light():
	for eye_angle in range(30, 150, 30):
		if red_seen(eye_angle):
			return True
	return False

if __name__ == '__main__':
	map_size = 11
	blast_radius = 3
	goal = (0, map_size // 2)
	step_size = 4
	
	try:
		print ('Program is starting ... ')
		sight_servo.setServoPwm('0',90)
		sight_servo.setServoPwm('1',100)
		while True:
			if not is_red_light():
				map_ = build_map(map_size, blast_radius)
				go_based_on_map(map_, map_size, goal, step_size)
	except KeyboardInterrupt:
		sight_servo.setServoPwm('0',90)
		sight_servo.setServoPwm('1',100)
		PWM.setMotorModel(0,0,0,0)
		print ("\nEnd of program")
