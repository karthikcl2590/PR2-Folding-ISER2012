#!/usr/bin/env python
import roslib
roslib.load_manifest("folding_geometry")
import rospy
import sys
from geometry_msgs.msg import PointStamped
from folding_msgs.msg import PolyStamped,Point2D
from numpy import *
import time
import tf

class PolyGenerator:
	def __init__(self):
		self.vertices = []
		self.points3D = []
		self.world_frame = rospy.get_param("~world_frame","odom_combined")
		self.poly_frame = rospy.get_param("~poly_frame_name","poly_frame")
		self.point_sub = rospy.Subscriber("input_3d_points",PointStamped,self.handle_incoming_point)
		self.poly_pub = rospy.Publisher("output_stamped_polygon",PolyStamped);
		self.listener = tf.TransformListener()
		
	def handle_incoming_point(self,stamped_point):
		#rospy.loginfo("generate_poly_from_points.py received point")
		now = rospy.Time.now()
		stamped_point.header.stamp = now
		self.listener.waitForTransform(self.world_frame,stamped_point.header.frame_id,now,rospy.Duration(10.0)) #Converting to world frame)
		world_point = self.listener.transformPoint(self.world_frame,stamped_point)
		self.points3D.append(world_point)
		if len(self.points3D) < 3:
			return
		elif len(self.points3D) == 3:
			self.init_polygon()
		else:
			self.update_polygon()
			self.publish_polygon()
		
	def init_polygon(self):
		#Starts with 3 points, constructs a polygon, calculates the frame
		##assert len(self.points3D)==3
		##[A,B,C] = [stamped_point.point for stamped_point in self.points3D]
		##AB = array([B.x-A.x,B.y-A.y,B.z-A.z])
		##AC = array([C.x-A.x,C.y-A.y,C.z-A.z])
		##crossproduct = cross(AB,AC)
		##self.normal_vector = crossproduct / sqrt(dot(crossproduct,crossproduct))
		self.make_origin(self.points3D[0])
		self.update_polygon()
		self.publish_polygon()
	
	def update_polygon(self):
		self.vertices = []
		transPts = self.points3D#[self.listener.transformPoint(self.world_frame,point) for point in self.points3D]
		for transPt in transPts:
			self.vertices.append(Point2D(x=-1*transPt.point.y,y=-1*transPt.point.x))
		self.z_offset = sum(stamped_point.point.z for stamped_point in transPts) / len(transPts)
	
	def make_origin(self,pt):
		#do nothing
		return

	def publish_polygon(self):
		vertices = [pt for pt in self.vertices]
		polygon = PolyStamped()
		polygon.vertices = vertices
		polygon.header.stamp = rospy.Time.now()
		polygon.header.frame_id = self.world_frame
		polygon.z_offset = self.z_offset
		self.poly_pub.publish(polygon)
		
def main(args):
	rospy.init_node("generate_poly_from_points")
	pg = PolyGenerator()
	rospy.spin()

if __name__ == '__main__':
	args = sys.argv[1:]
	try:
		main(args)
	except rospy.ROSInterruptException: pass
