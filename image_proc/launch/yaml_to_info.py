#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image

def yaml_to_CameraInfo(yaml_fname):
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = 'camera_fisheye_front_optical'
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

image_sub = None
info_pub = None
image_pub = None

def image_cb(msg):
    camera_info_msg.header.stamp = msg.header.stamp
    info_pub.publish(camera_info_msg)
    image_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("camera_info_publisher", anonymous=True)
    
    import argparse
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("filename", help="Path to yaml file containing " +\
                                             "camera calibration data")
    args = arg_parser.parse_args()
    filename = args.filename

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    info_pub = rospy.Publisher("camera_info_synched", CameraInfo, queue_size=1)
    image_pub = rospy.Publisher("image_raw_synched", Image, queue_size=1)
    image_sub = rospy.Subscriber("image_raw", Image, image_cb)
    
    rospy.spin()
            
