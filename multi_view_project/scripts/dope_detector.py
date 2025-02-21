#!/usr/bin/env python3
import rospy
from vision_msgs.msg import Detection3DArray

def callback(data):
    rospy.loginfo("Number of detected objects: %d", len(data.detections))
    rospy.loginfo("Total Objects: %d", len(data.detections))

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


def dope_detector():
    rospy.init_node('dope_detector', anonymous=True)
    rospy.Subscriber("/dope/detected_objects", Detection3DArray, callback)
    rospy.spin()

if __name__ == '__main__':
    dope_detector()

