#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_multiply

class DopeDetector:
    def __init__(self):
        rospy.init_node('dope_detector', anonymous=True)
        
        # Initialize TransformListener and broadcaster
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Marker publisher for RViz visualization
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        
        rospy.Subscriber("/dope/detected_objects", Detection3DArray, self.callback)
        rospy.spin()

    def callback(self, data):
        rospy.loginfo("Number of detected objects: %d", len(data.detections))

        try:
            self.tf_listener.waitForTransform("/world", "/camera_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform("/world", "/camera_link", rospy.Time(0))
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
            rospy.loginfo(f"  - Position (Camera Frame): x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}")
            rospy.loginfo(f"  - Orientation: x={orientation.x:.3f}, y={orientation.y:.3f}, z={orientation.z:.3f}, w={orientation.w:.3f}")

            # **Rotate Position by 180° Around X-axis**
            rotated_x = position.x
            rotated_y = -position.y
            rotated_z = -position.z

            # **Rotate Orientation by 180° Around X-axis**
            # Quaternion for 180° rotation around X-axis
            q_rotation = tf.transformations.quaternion_from_euler(0, 0, 0)  #(red, blue, green) 3.1415/2, 3.1415, -3.1415/2
            rotated_orientation = quaternion_multiply([orientation.x, orientation.y, orientation.z, orientation.w], q_rotation)

            # Create a PoseStamped message
            obj_pose = PoseStamped()
            obj_pose.header.frame_id = "camera_link"
            obj_pose.header.stamp = rospy.Time(0)
            obj_pose.pose.position.x = rotated_x
            obj_pose.pose.position.y = rotated_y
            obj_pose.pose.position.z = rotated_z
            obj_pose.pose.orientation.x = rotated_orientation[0]
            obj_pose.pose.orientation.y = rotated_orientation[1]
            obj_pose.pose.orientation.z = rotated_orientation[2]
            obj_pose.pose.orientation.w = rotated_orientation[3]

            # Transform to world frame
            try:
                obj_pose_world = self.tf_listener.transformPose("/world", obj_pose)
                world_pos = obj_pose_world.pose.position
                world_ori = obj_pose_world.pose.orientation
                rospy.loginfo(f"Object {i+1} in World Frame: x={world_pos.x:.3f}, y={world_pos.y:.3f}, z={world_pos.z:.3f}")

                # Broadcast the detected object's pose in the world frame
                self.tf_broadcaster.sendTransform(
                    (world_pos.x, world_pos.y, world_pos.z), 
                    (world_ori.x, world_ori.y, world_ori.z, world_ori.w),
                    rospy.Time.now(),
                    f"detected_object_{obj_id}",
                    "world"
                )

                # Publish Marker for visualization in RViz
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "object"
                marker.id = obj_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # Set position
                marker.pose.position.x = world_pos.x
                marker.pose.position.y = world_pos.y
                marker.pose.position.z = world_pos.z

                # Set orientation
                marker.pose.orientation.x = world_ori.x
                marker.pose.orientation.y = world_ori.y
                marker.pose.orientation.z = world_ori.z
                marker.pose.orientation.w = world_ori.w

                # Set Marker size
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                # Set Marker color (red)
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0  

                marker.lifetime = rospy.Duration()
                self.marker_pub.publish(marker)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"Transformation failed: {e}")

if __name__ == '__main__':
    DopeDetector()

