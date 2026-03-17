import math
import numpy as np
from utils import *
from wall import Wall

class LidarSim:
   # Constructor
   def __init__(self, walls:list[Wall], max_range:float, n_rays:float):
      self.walls = walls
      self.max_range = max_range
      self.n_rays = n_rays
      self.resolution = int(360/n_rays)
      self.measurements = math.inf*np.ones(self.resolution)

   # Simulate the lidar sensor reading
   def read(self, pose:SE2) -> np.ndarray:
      '''
      Simulate the lidar sensor readings given the current pose of the robot.  Please see the lab handout for more details and helpful figures.

      Parameters:
      pose (SE2): The current pose of the robot, represented as an SE2 object, 
      which includes the x and y coordinates and the heading (orientation) of 
      the robot.

      Returns:
      np.ndarray: An array of simulated lidar measurements, where each element 
      represents the distance to the nearest wall for a specific lidar ray.

      Steps:
      1. Iterate through each lidar ray:
         - For each ray, calculate the angle based on the robot's heading and 
         the resolution of the lidar.
         - Determine the endpoint of the lidar ray based on the maximum range 
         and the calculated angle.

      2. Check for intersections with walls:
         - For each wall, check if the lidar ray intersects with the wall 
         using the line_rectangle_intersect function.
         - If an intersection is detected, calculate the intersection points 
         between the lidar ray and the edges of the wall.
         - Calculate the distances from the robot to these intersection points.

      4. Find the minimum distance:
         - Among all intersection points, find the minimum distance and update 
         the measurements array for the corresponding ray.

      5. Return the measurements:
         - Return the array of simulated lidar measurements.
      '''

      # Reset the measurements
      self.measurements = math.inf*np.ones(self.n_rays) # Webots lidar sensor returns inf for no detection
      
      ######### START STUDENT CODE #########
      for i in range(int(self.n_rays)):
         angle = pose.h + (2 * math.pi * (i + 0.5) / self.n_rays)

         ray_start = pose.position()
         ray_end = Point(
            ray_start.x + self.max_range * math.cos(angle),
            ray_start.y + self.max_range * math.sin(angle)
         )

         for wall in self.walls:
            if not line_rectangle_intersect(ray_start, ray_end, wall.pose, wall.dimensions):
                  continue

            # Build corners using dimensions[0]=width, dimensions[1]=height
            a = wall.dimensions[0] / 2
            b = wall.dimensions[1] / 2

            p1_l = Point(-a, -b)
            p2_l = Point( a, -b)
            p3_l = Point( a,  b)
            p4_l = Point(-a,  b)

            p1_w = wall.pose.transform_point(p1_l)
            p2_w = wall.pose.transform_point(p2_l)
            p3_w = wall.pose.transform_point(p3_l)
            p4_w = wall.pose.transform_point(p4_l)

            edges = [(p1_w, p2_w), (p2_w, p3_w), (p3_w, p4_w), (p4_w, p1_w)]

            for e1, e2 in edges:
                  pt = line_intersection(ray_start, ray_end, e1, e2)
                  if pt is not None:
                     dist = distance_between_points(ray_start, pt)
                     if dist < self.measurements[i]:
                        self.measurements[i] = dist
      ########## END STUDENT CODE ##########

      return self.measurements
