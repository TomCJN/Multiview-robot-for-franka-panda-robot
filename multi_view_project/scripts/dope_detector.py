#!/usr/bin/env python3
import rospy
import tf
from vision_msgs.msg import Detection3DArray

def callback(data):
    rospy.loginfo("Number of detected objects: %d", len(data.detections))
    rospy.loginfo("Total Objects: %d", len(data.detections))
    
    tf_listener = tf.TransformListener()

    try:
        tf_listener.waitForTransform("/world", "/camera_link", rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = tf_listener.lookupTransform("/world", "/camera_link", rospy.Time(0))

        rospy.loginfo(f"World to Camera Transform: Translation = {trans}, Rotation = {rot}")

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Could not get world to camera transform")
        return
    
    
    for i, detection in enumerate(data.detections):
        obj_id = detection.results[0].id
        confidence_score = detection.results[0].score
        position = detection.results[0].pose.pose.position
        orientation = detection.results[0].pose.pose.orientation

        rospy.loginfo(f"Object {i+1}:")
        rospy.loginfo(f"  - ID: {obj_id}")
        rospy.loginfo(f"  - Confidence Score: {confidence_score:.3f}")
        rospy.loginfo(f"  - Position: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}")
        rospy.loginfo(f"  - Orientation: x={orientation.x:.3f}, y={orientation.y:.3f}, z={orientation.z:.3f}, w={orientation.w:.3f}")
        
        obj_trans = (position.x, position.y, position.z)
        obj_rot = (orientation.x, orientation.y, orientation.z, orientation.w)

        obj_in_world = tf_listener.fromTranslationRotation(obj_trans, obj_rot)

        rospy.loginfo(f"Object {i+1} in World Frame: {obj_in_world}")


def dope_detector():
    rospy.init_node('dope_detector', anonymous=True)
    rospy.Subscriber("/dope/detected_objects", Detection3DArray, callback)
    rospy.spin()

if __name__ == '__main__':
    dope_detector()

