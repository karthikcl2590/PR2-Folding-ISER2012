#!/usr/bin/env python

import roslib
roslib.load_manifest("folding_vision")
import rospy
import sys
import tf
from geometry_msgs.msg import PointStamped
from numpy import *
from folding_vision.srv import *
from folding_srvs.srv import *
from clothing_models import Models
import pickle
import image_geometry
from image_processor.srv import *
from sensor_msgs.msg import CameraInfo
import StanceUtils
from rll_utils import RosUtils


## This adds visual feedback to the folding procedure, providing services to determine the cloth polygon,
#  and adjust fold lines to match the observed ones. Currently made to interface with the FoldingGUI,
#  though it could certainly be made more modular
class VisionBridgeNode:
    def __init__(self):
        self.name = rospy.get_name()
        self.listener = tf.TransformListener()
        locate_poly_srv = rospy.Service("%s/locate_polygon"%self.name,LocatePolygon,self.locate_polygon)
        adjust_fold_srv = rospy.Service("adjust_fold",AdjustFold,self.adjust_fold)
        self.pt_pub = rospy.Publisher("output_pts",PointStamped)

    def publish(self,pt):
        self.pt_pub.publish(pt)

    # Assuming I am looking at an article of clothing on a green background,
    # find it and all relevant vertices. 
    def locate_polygon(self,req):
        #Go to viewing stance
        StanceUtils.call_stance("open_both",2.0)
        StanceUtils.call_stance("viewing",2.0)
        locate_poly = rospy.ServiceProxy("shape_fitter_node/process_mono",ProcessMono)
        print "Calling locate_poly"
        resp = locate_poly("wide_stereo/left")
        print "Received response"
        pts = resp.pts3d
        for pt in pts:
            self.publish(pt)
        return LocatePolygonResponse()
        
    def adjust_fold(self,req):
        #Go to viewing stance
        StanceUtils.call_stance("open_both",5.0)
        StanceUtils.call_stance("viewing",5.0)
        last_model = pickle.load(open("/tmp/last_model.pickle"))
        camera_model = image_geometry.PinholeCameraModel()
        info = RosUtils.get_next_message("wide_stereo/left/camera_info",CameraInfo)
        cam_frame = info.header.frame_id
        camera_model.fromCameraInfo(info)
        now = rospy.Time.now()
        req.start.header.stamp = now
        req.end.header.stamp=now
        self.listener.waitForTransform(cam_frame,req.start.header.frame_id,now,rospy.Duration(20.0))
        start_cam = self.listener.transformPoint(cam_frame,req.start)
        end_cam = self.listener.transformPoint(cam_frame,req.end)
        start = camera_model.project3dToPixel((start_cam.point.x,start_cam.point.y,start_cam.point.z))
        end = camera_model.project3dToPixel((end_cam.point.x,end_cam.point.y,end_cam.point.z))
        folded_model = Models.Point_Model_Folded(last_model,start,end)
        folded_model.image = None
        folded_model.initial_model.image = None
        pickle.dump(folded_model,open("/tmp/last_model.pickle",'w'))
        adjust_folded_model = rospy.ServiceProxy("fold_finder_node/process_mono",ProcessMono)
        resp = adjust_folded_model("wide_stereo/left")
        new_start = resp.pts3d[0]
        new_end = resp.pts3d[1]
        return AdjustFoldResponse(start=new_start,end=new_end)


def main(args):
    rospy.init_node("folding_vision_bridge_node")
    vbn = VisionBridgeNode()
    rospy.spin()
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
