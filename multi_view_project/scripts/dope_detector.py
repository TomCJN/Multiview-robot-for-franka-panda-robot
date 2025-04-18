#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix, euler_from_matrix, quaternion_multiply
import tf.transformations
import numpy as np

class DopeDetector:
    def __init__(self):
        rospy.init_node('dope_detector', anonymous=True)

        # Initialize TransformListener and broadcaster
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Marker publishers for RViz visualization
        self.marker_cam_pub = rospy.Publisher('camera_frame_marker', Marker, queue_size=10)
        self.marker_world_pub = rospy.Publisher('world_frame_marker', Marker, queue_size=10)

        rospy.Subscriber("/dope/detected_objects", Detection3DArray, self.callback)
        rospy.spin()

    def callback(self, data):
        rospy.loginfo("Number of detected objects: %d", len(data.detections))

        try:
            self.tf_listener.waitForTransform("/world", "/camera_link_optical", rospy.Time(0), rospy.Duration(1.0))
            (trans_world_to_cam, rot_world_to_cam) = self.tf_listener.lookupTransform("/world", "/camera_link_optical", rospy.Time(0))

            matrix_world_to_cam = concatenate_matrices(translation_matrix(trans_world_to_cam), quaternion_matrix(rot_world_to_cam))
            rospy.loginfo(f"\n--- World to Camera Transformation")
            formatted_matrix = np.array2string(matrix_world_to_cam, precision=4, floatmode='fixed', suppress_small=True)
            rospy.loginfo(formatted_matrix)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Could not get world to camera transform")
            return

        for i, detection in enumerate(data.detections):
            obj_id = detection.results[0].id
            confidence_score = detection.results[0].score
            position = detection.results[0].pose.pose.position
            orientation = detection.results[0].pose.pose.orientation

            #rospy.loginfo(f"\n--- Detected Object {i+1} (Camera Frame) ---")
            #rospy.loginfo(f"  Position: x={position.x:.4f}, y={position.y:.4f}, z={position.z:.4f}")
            #rospy.loginfo(f"  Orientation (Quaternion): x={orientation.x:.4f}, y={orientation.y:.4f}, z={orientation.z:.4f}, w={orientation.w:.4f}")

            # **Rotate Position by 180° Around X-axis**
            rotated_x = position.x
            rotated_y = position.y
            rotated_z = position.z

            # **Rotate Orientation by 180° Around X-axis**
            q_rotation = tf.transformations.quaternion_from_euler(0, 0, 0)
            rotated_orientation = tf.transformations.quaternion_multiply([orientation.x, orientation.y, orientation.z, orientation.w], q_rotation)

            rospy.loginfo(f"\n--- Camera to Object Transformation")
            matrix_cam_to_obj_rotated = concatenate_matrices(translation_matrix((rotated_x, rotated_y, rotated_z)), quaternion_matrix(rotated_orientation))
            rospy.loginfo(np.array2string(matrix_cam_to_obj_rotated, precision=4, floatmode='fixed'))

            self.tf_broadcaster.sendTransform(
                (position.x, position.y, position.z),
                (orientation.x, orientation.y, orientation.z, orientation.w),
                rospy.Time.now(),
                f"detected_object_{obj_id}_camera_frame",
                "camera_link_optical"
            )

            # --- Publish Marker in Camera Frame (Red) ---
            marker_cam = Marker()
            marker_cam.header.frame_id = "camera_link_optical"
            marker_cam.header.stamp = rospy.Time.now()
            marker_cam.ns = "object_camera_frame"
            marker_cam.id = obj_id * 3
            marker_cam.type = Marker.CUBE
            marker_cam.action = Marker.ADD
            marker_cam.pose.position.x = rotated_x
            marker_cam.pose.position.y = rotated_y
            marker_cam.pose.position.z = rotated_z
            marker_cam.pose.orientation.x = rotated_orientation[0]
            marker_cam.pose.orientation.y = rotated_orientation[1]
            marker_cam.pose.orientation.z = rotated_orientation[2]
            marker_cam.pose.orientation.w = rotated_orientation[3]
            marker_cam.scale.x = 0.1
            marker_cam.scale.y = 0.1
            marker_cam.scale.z = 0.1
            marker_cam.color.r = 1.0
            marker_cam.color.g = 0.0
            marker_cam.color.b = 0.0
            marker_cam.color.a = 0.7
            marker_cam.lifetime = rospy.Duration()
            self.marker_cam_pub.publish(marker_cam)

            # --- Calculate World Frame Directly and Publish Marker (Green) ---
            matrix_world_to_obj_direct = np.dot(matrix_world_to_cam, matrix_cam_to_obj_rotated)
            world_pos_direct = translation_from_matrix(matrix_world_to_obj_direct)
            world_ori_direct = quaternion_from_matrix(matrix_world_to_obj_direct)

            marker_world_direct = Marker()
            marker_world_direct.header.frame_id = "world"
            marker_world_direct.header.stamp = rospy.Time.now()
            marker_world_direct.ns = "object_world_frame_direct"
            marker_world_direct.id = obj_id * 3 + 1
            marker_world_direct.type = Marker.CUBE
            marker_world_direct.action = Marker.ADD
            marker_world_direct.pose.position.x = world_pos_direct[0]
            marker_world_direct.pose.position.y = world_pos_direct[1]
            marker_world_direct.pose.position.z = world_pos_direct[2]
            marker_world_direct.pose.orientation.x = world_ori_direct[0]
            marker_world_direct.pose.orientation.y = world_ori_direct[1]
            marker_world_direct.pose.orientation.z = world_ori_direct[2]
            marker_world_direct.pose.orientation.w = world_ori_direct[3]
            marker_world_direct.scale.x = 0.1
            marker_world_direct.scale.y = 0.1
            marker_world_direct.scale.z = 0.1
            marker_world_direct.color.r = 0.0
            marker_world_direct.color.g = 1.0
            marker_world_direct.color.b = 0.0
            marker_world_direct.color.a = 0.7
            marker_world_direct.lifetime = rospy.Duration()
            self.marker_world_pub.publish(marker_world_direct)

            # --- Publish Marker in World Frame (Blue - using transformPose) ---
            obj_pose_cam = PoseStamped()
            obj_pose_cam.header.frame_id = "camera_link_optical"
            obj_pose_cam.header.stamp = rospy.Time(0)
            obj_pose_cam.pose.position.x = rotated_x
            obj_pose_cam.pose.position.y = rotated_y
            obj_pose_cam.pose.position.z = rotated_z
            obj_pose_cam.pose.orientation.x = rotated_orientation[0]
            obj_pose_cam.pose.orientation.y = rotated_orientation[1]
            obj_pose_cam.pose.orientation.z = rotated_orientation[2]
            obj_pose_cam.pose.orientation.w = rotated_orientation[3]

            try:
                obj_pose_world = self.tf_listener.transformPose("/world", obj_pose_cam)
                world_pos = obj_pose_world.pose.position
                world_ori = obj_pose_world.pose.orientation
                rospy.loginfo(f"\n--- World to Object Transformation")
                matrix_world_to_obj_tf = concatenate_matrices(translation_matrix((world_pos.x, world_pos.y, world_pos.z)), quaternion_matrix((world_ori.x, world_ori.y, world_ori.z, world_ori.w)))
                rospy.loginfo(np.array2string(matrix_world_to_obj_tf, precision=4, floatmode='fixed'))

                # Broadcast the detected object's pose in the world frame
                self.tf_broadcaster.sendTransform(
                    (world_pos.x, world_pos.y, world_pos.z),
                    (world_ori.x, world_ori.y, world_ori.z, world_ori.w),
                    rospy.Time.now(),
                    f"detected_object_{obj_id}_world_frame",
                    "world"
                )

                marker_world = Marker()
                marker_world.header.frame_id = "world"
                marker_world.header.stamp = rospy.Time.now()
                marker_world.ns = "object_world_frame_transformed"
                marker_world.id = obj_id * 3 + 2
                marker_world.type = Marker.CUBE
                marker_world.action = Marker.ADD
                marker_world.pose.position = world_pos
                marker_world.pose.orientation = world_ori
                marker_world.scale.x = 0.1
                marker_world.scale.y = 0.1
                marker_world.scale.z = 0.1
                marker_world.color.r = 0.0
                marker_world.color.g = 0.0
                marker_world.color.b = 1.0
                marker_world.color.a = 0.7
                marker_world.lifetime = rospy.Duration()
                self.marker_world_pub.publish(marker_world)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"Could not transform object to world frame for marker: {e}")

if __name__ == '__main__':
    DopeDetector()
