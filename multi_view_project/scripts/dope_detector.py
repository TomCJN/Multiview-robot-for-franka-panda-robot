#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
import tf.transformations
from itertools import combinations

class DopeDetectorMultiViewComparison:
    def __init__(self):
        rospy.init_node('dope_detector_multiview_comparison', anonymous=True)

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/dope/detected_objects", Detection3DArray, self.callback)

        self.target_view_matrices = {
            "viewpoint_1": np.array([
                [ 0.6577,  0.0000,  0.7533,  0.2167],
                [-0.0000, -1.0000,  0.0000, -0.0000],
                [ 0.7533, -0.0000, -0.6577,  0.5137],
                [ 0.0000,  0.0000,  0.0000,  1.0000]
            ]),
            "viewpoint_2": np.array([
                [-0.2587, -0.9077,  0.3303,  0.4850],
                [-0.4794,  0.4175,  0.7719, -0.4583],
                [-0.8386,  0.0413, -0.5432,  0.4182],
                [ 0.0000,  0.0000,  0.0000,  1.0000]
            ]),
            "viewpoint_3": np.array([
                [-1.0000, -0.0003, -0.0021,  0.6638],
                [-0.0003,  1.0000, -0.0000,  0.0002],
                [ 0.0021, -0.0000, -1.0000,  0.5678],
                [ 0.0000,  0.0000,  0.0000,  1.0000]
            ]),
            "viewpoint_4": np.array([
                [-0.2897,  0.9008,  0.3234,  0.4971],
                [ 0.5105,  0.4312, -0.7439,  0.4538],
                [-0.8096, -0.0504, -0.5848,  0.4494],
                [ 0.0000,  0.0000,  0.0000,  1.0000]
            ]),
        }
        self.current_viewpoint = None
        self.detections_at_viewpoint = {}  # {viewpoint: {object_id: [positions]}}
        self.viewpoint_start_time = {}
        self.max_detections_per_object = 3 # Changed from max_detections_per_viewpoint
        self.max_empty_detections_threshold = 4
        self.empty_detection_counts = {vp: 0 for vp in self.target_view_matrices}
        self.camera_optical_frame = "camera_link_optical"
        self.world_frame = "world"
        self.viewpoint_tolerance_translation = 0.1
        self.viewpoint_tolerance_rotation = 0.1
        self.last_camera_pose = None
        self.min_movement_for_new_detection = 0.02
        self.viewpoint_reached = {vp: False for vp in self.target_view_matrices}
        self.object_detections_collected = {vp: {} for vp in self.target_view_matrices} # {viewpoint: {object_id: count}}
        self.average_positions = {}  # {object_id: {viewpoint: [positions]}}
        self.all_viewpoints_done = False

        rospy.spin()

    def get_camera_pose(self):
        try:
            self.tf_listener.waitForTransform(self.world_frame, self.camera_optical_frame, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, self.camera_optical_frame, rospy.Time(0))
            return np.array(trans), np.array(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Could not get camera pose in world frame.")
            return None, None

    def is_at_viewpoint(self, current_translation, current_rotation, target_matrix, trans_tolerance, rot_tolerance):
        target_translation = target_matrix[:3, 3]
        target_rotation_quat = tf.transformations.quaternion_from_matrix(target_matrix)

        translation_diff_norm = np.linalg.norm(current_translation - target_translation)

        # Calculate the quaternion difference
        quat_diff = tf.transformations.quaternion_multiply(
            current_rotation, tf.transformations.quaternion_inverse(target_rotation_quat)
        )

        # The magnitude of the vector part of the quaternion difference can be used as a measure of rotational difference
        rotational_difference = np.linalg.norm(quat_diff[:3])

        return translation_diff_norm <= trans_tolerance and rotational_difference <= rot_tolerance

    def has_moved(self, current_trans, current_rot, last_trans, last_rot):
        if last_trans is None or last_rot is None:
            return True

        trans_diff = np.linalg.norm(current_trans - last_trans)
        if last_rot is not None:
            rot_diff_quat = tf.transformations.quaternion_multiply(
                current_rot, tf.transformations.quaternion_inverse(last_rot)
            )
            angular_movement = np.linalg.norm(rot_diff_quat[:3])
        else:
            angular_movement = float('inf')

        return trans_diff > self.min_movement_for_new_detection or angular_movement > self.min_movement_for_new_detection

    def callback(self, data):
        rospy.loginfo(f"Received detection data: {len(data.detections)} detections")
        if self.all_viewpoints_done:
            rospy.loginfo("All viewpoints processed, ignoring new data.")
            return

        camera_translation, camera_rotation = self.get_camera_pose()
        if camera_translation is not None and camera_rotation is not None:
            rospy.loginfo("--- Current World to camera_link_optical Transform ---")
            rospy.loginfo(f"Translation: {camera_translation}")
            current_euler = tf.transformations.euler_from_quaternion(camera_rotation)
            rospy.loginfo(f"Euler Angles (XYZ): {current_euler}")
            rospy.loginfo("--------------------------------------------------")

        if camera_translation is None:
            rospy.logwarn("Could not get camera pose, skipping callback.")
            return

        at_least_one_viewpoint_active = False
        for name, target_matrix in self.target_view_matrices.items():
            at_viewpoint = self.is_at_viewpoint(camera_translation, camera_rotation, target_matrix,
                                                self.viewpoint_tolerance_translation, self.viewpoint_tolerance_rotation)
            rospy.loginfo(f"Checking viewpoint {name}: at_viewpoint={at_viewpoint}, object_detections_collected={self.object_detections_collected[name]}, empty_detections={self.empty_detection_counts[name]}")

            if at_viewpoint:
                at_least_one_viewpoint_active = True
                self.current_viewpoint = name
                if not self.viewpoint_reached[name]:
                    rospy.loginfo(f"Arrived at viewpoint: {name}")
                    self.detections_at_viewpoint[name] = {}  # Initialize for this viewpoint
                    self.viewpoint_start_time[name] = rospy.Time.now().to_sec()
                    self.last_camera_pose = (camera_translation.copy(), camera_rotation.copy())
                    self.viewpoint_reached[name] = True
                    self.empty_detection_counts[name] = 0
                    self.object_detections_collected[name] = {} # Initialize object detection counts for this viewpoint
                break
            elif self.viewpoint_reached[name]:
                self.current_viewpoint = name

        if self.current_viewpoint is not None and self.viewpoint_reached[self.current_viewpoint]:
            if not data.detections:
                self.empty_detection_counts[self.current_viewpoint] += 1
                rospy.logwarn(f"Received empty detection array at {self.current_viewpoint}. Consecutive empty detections: {self.empty_detection_counts[self.current_viewpoint]}")
                if self.empty_detection_counts[self.current_viewpoint] >= self.max_empty_detections_threshold:
                    rospy.loginfo(f"Reached max empty detections at {self.current_viewpoint}. Finishing data collection.")
                    self.finish_viewpoint_collection(self.current_viewpoint)
                    self.current_viewpoint = None
                    return
            else:
                self.empty_detection_counts[self.current_viewpoint] = 0
                camera_pose = (camera_translation.copy(), camera_rotation.copy())
                if self.is_at_viewpoint(camera_translation, camera_rotation, self.target_view_matrices[self.current_viewpoint],
                                        self.viewpoint_tolerance_translation, self.viewpoint_tolerance_rotation):
                    rospy.loginfo(f"Processing detection at {self.current_viewpoint}")

                    # Process detections for each object
                    object_detections = {}
                    for detection in data.detections:
                        obj_id = detection.results[0].id
                        if obj_id not in object_detections:
                            object_detections[obj_id] = []
                        object_detections[obj_id].append(detection)

                    for obj_id, detections in object_detections.items():
                        if obj_id not in self.object_detections_collected[self.current_viewpoint]:
                            self.object_detections_collected[self.current_viewpoint][obj_id] = 0

                        if self.object_detections_collected[self.current_viewpoint][obj_id] < self.max_detections_per_object:
                            rospy.loginfo(f"  Processing object ID: {obj_id} ({self.object_detections_collected[self.current_viewpoint][obj_id]}/{self.max_detections_per_object})")
                            for detection in detections:
                                position = detection.results[0].pose.pose.position
                                orientation = detection.results[0].pose.pose.orientation

                                obj_pose_cam = PoseStamped()
                                obj_pose_cam.header.frame_id = self.camera_optical_frame
                                obj_pose_cam.header.stamp = rospy.Time(0)
                                obj_pose_cam.pose.position = position
                                obj_pose_cam.pose.orientation = orientation

                                try:
                                    obj_pose_world = self.tf_listener.transformPose(self.world_frame, obj_pose_cam)
                                    world_pos = obj_pose_world.pose.position
                                    if obj_id not in self.detections_at_viewpoint[self.current_viewpoint]:
                                        self.detections_at_viewpoint[self.current_viewpoint][obj_id] = []
                                    self.detections_at_viewpoint[self.current_viewpoint][obj_id].append(
                                        np.array([world_pos.x, world_pos.y, world_pos.z]))
                                    rospy.loginfo(
                                        f"   Detected object {obj_id} (world): x={world_pos.x:.4f}, y={world_pos.y:.4f}, z={world_pos.z:.4f}")

                                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                                    rospy.logwarn(f"Could not transform object to world frame: {e}")
                            self.object_detections_collected[self.current_viewpoint][obj_id] += 1

                            if self.object_detections_collected[self.current_viewpoint][obj_id] >= self.max_detections_per_object:
                                rospy.loginfo(f"   Collected enough detections for object {obj_id} at {self.current_viewpoint}")
                                # Check if all objects have enough detections
                                all_objects_done = True
                                for obj_id_check in self.object_detections_collected[self.current_viewpoint]:
                                    if self.object_detections_collected[self.current_viewpoint][obj_id_check] < self.max_detections_per_object:
                                        all_objects_done = False
                                        break
                                if all_objects_done:
                                    self.finish_viewpoint_collection(self.current_viewpoint)
                                    self.current_viewpoint = None
                                    return
                    self.last_camera_pose = camera_pose
                else:
                    moved = self.has_moved(camera_translation, camera_rotation,
                                            self.last_camera_pose[0] if self.last_camera_pose is not None else None,
                                            self.last_camera_pose[1] if self.last_camera_pose is not None else None)
                    rospy.loginfo(f"At viewpoint {self.current_viewpoint} (but checking movement): moved={moved}")
                    if moved:
                        rospy.loginfo(f"Processing detection at {self.current_viewpoint} (moved after being at viewpoint)")
                        # Process detections for each object
                        object_detections = {}
                        for detection in data.detections:
                            obj_id = detection.results[0].id
                            if obj_id not in object_detections:
                                object_detections[obj_id] = []
                            object_detections[obj_id].append(detection)

                        for obj_id, detections in object_detections.items():
                            if obj_id not in self.object_detections_collected[self.current_viewpoint]:
                                self.object_detections_collected[self.current_viewpoint][obj_id] = 0
                            if self.object_detections_collected[self.current_viewpoint][obj_id] < self.max_detections_per_object:
                                rospy.loginfo(f"  Processing object ID: {obj_id} ({self.object_detections_collected[self.current_viewpoint][obj_id]}/{self.max_detections_per_object})")
                                for detection in detections:
                                    position = detection.results[0].pose.pose.position
                                    orientation = detection.results[0].pose.pose.orientation

                                    obj_pose_cam = PoseStamped()
                                    obj_pose_cam.header.frame_id = self.camera_optical_frame
                                    obj_pose_cam.header.stamp = rospy.Time(0)
                                    obj_pose_cam.pose.position = position
                                    obj_pose_cam.pose.orientation = orientation

                                    try:
                                        obj_pose_world = self.tf_listener.transformPose(self.world_frame, obj_pose_cam)
                                        world_pos = obj_pose_world.pose.position
                                        if obj_id not in self.detections_at_viewpoint[self.current_viewpoint]:
                                            self.detections_at_viewpoint[self.current_viewpoint][obj_id] = []
                                        self.detections_at_viewpoint[self.current_viewpoint][obj_id].append(
                                            np.array([world_pos.x, world_pos.y, world_pos.z]))
                                        rospy.loginfo(
                                            f"   Detected object {obj_id} (world): x={world_pos.x:.4f}, y={world_pos.y:.4f}, z={world_pos.z:.4f}")

                                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                                        rospy.logwarn(f"Could not transform object to world frame: {e}")
                                self.object_detections_collected[self.current_viewpoint][obj_id] += 1
                                if self.object_detections_collected[self.current_viewpoint][obj_id] >= self.max_detections_per_object:
                                    rospy.loginfo(f"   Collected enough detections for object {obj_id} at {self.current_viewpoint}")
                                    # Check if all objects have enough detections
                                    all_objects_done = True
                                    for obj_id_check in self.object_detections_collected[self.current_viewpoint]:
                                        if self.object_detections_collected[self.current_viewpoint][obj_id_check] < self.max_detections_per_object:
                                            all_objects_done = False
                                            break
                                    if all_objects_done:
                                        self.finish_viewpoint_collection(self.current_viewpoint)
                                        self.current_viewpoint = None
                                        return
        elif at_least_one_viewpoint_active:
            if self.current_viewpoint is not None and not self.is_at_viewpoint(camera_translation, camera_rotation,
                                                                            self.target_view_matrices[
                                                                                self.current_viewpoint],
                                                                            self.viewpoint_tolerance_translation,
                                                                            self.viewpoint_tolerance_rotation):
                self.empty_detection_counts[self.current_viewpoint] = 0
                self.current_viewpoint = None
        elif not at_least_one_viewpoint_active:
            self.current_viewpoint = None

    def finish_viewpoint_collection(self, viewpoint_name):
        if viewpoint_name not in self.detections_at_viewpoint:
            rospy.logwarn(f"No detections collected at {viewpoint_name}, cannot calculate average.")
            self.check_all_viewpoints_done()
            return

        for obj_id, positions in self.detections_at_viewpoint[viewpoint_name].items():
            if obj_id not in self.average_positions:
                self.average_positions[obj_id] = {}
            if positions:
                average_position = np.mean(np.array(positions), axis=0)
                self.average_positions[obj_id][viewpoint_name] = average_position
                rospy.loginfo(f"\n--- Average Position for object {obj_id} at {viewpoint_name} ---")
                rospy.loginfo(f"  Average Position: x={average_position[0]:.4f}, y={average_position[1]:.4f}, z={average_position[2]:.4f}")
            else:
                rospy.logwarn(f"No detections collected for object {obj_id} at {viewpoint_name}, cannot calculate average.")
                self.average_positions[obj_id][viewpoint_name] = None
        rospy.loginfo(f"Finished collecting data at viewpoint: {viewpoint_name}")
        self.check_all_viewpoints_done()

    def check_all_viewpoints_done(self):
        rospy.loginfo(f"Checking if all viewpoints are done. viewpoints reached: {self.viewpoint_reached}")
        if all(self.viewpoint_reached.values()):
            rospy.loginfo("All viewpoints have been reached. Outputting comparison averages.")
            self.output_comparison_averages()
            self.all_viewpoints_done = True
            rospy.signal_shutdown("Finished processing all viewpoints.")

    def calculate_average_of_views(self, object_id, view_names):
        positions = [self.average_positions[object_id][vp] for vp in view_names
                     if object_id in self.average_positions and vp in self.average_positions[object_id] and
                     self.average_positions[object_id][vp] is not None]
        if positions:
            return np.mean(np.array(positions), axis=0)
        return None

    def output_comparison_averages(self):
        rospy.loginfo("\n--- Comparison of Averages ---")
        object_ids = set(self.average_positions.keys())

        for obj_id in object_ids:
            rospy.loginfo(f"\n--- Object ID: {obj_id} ---")
            rospy.loginfo("\n--- Individual View Averages ---")
            for vp in self.target_view_matrices:
                if vp in self.average_positions[obj_id] and self.average_positions[obj_id][vp] is not None:
                    avg = self.average_positions[obj_id][vp]
                    rospy.loginfo(f"  Average of {vp}: x={avg[0]:.4f}, y={avg[1]:.4f}, z={avg[2]:.4f}")
                else:
                    rospy.loginfo(f"  Average for {vp} not available.")

            rospy.loginfo("\n--- Averages of Pairs of Views ---")
            for pair in combinations(self.target_view_matrices.keys(), 2):
                avg = self.calculate_average_of_views(obj_id, pair)
                if avg is not None:
                    rospy.loginfo(f"  Average of {pair[0]} + {pair[1]}: x={avg[0]:.4f}, y={avg[1]:.4f}, z={avg[2]:.4f}")
                else:
                    rospy.loginfo(
                        f"  Average for pair {pair[0]} + {pair[1]} not available (missing data from one or both).")

            rospy.loginfo("\n--- Averages of Triplets of Views ---")
            for triplet in combinations(self.target_view_matrices.keys(), 3):
                avg = self.calculate_average_of_views(obj_id, triplet)
                if avg is not None:
                    rospy.loginfo(
                        f"  Average of {triplet[0]} + {triplet[1]} + {triplet[2]}: x={avg[0]:.4f}, y={avg[1]:.4f}, z={avg[2]:.4f}")
                else:
                    rospy.loginfo(
                        f"  Average for triplet {triplet[0]} + {triplet[1]} + {triplet[2]} not available (missing data from one or more).")

            # Full system for each object.
            valid_averages = [self.average_positions[obj_id][vp] for vp in self.target_view_matrices if
                              vp in self.average_positions[obj_id] and self.average_positions[obj_id][vp] is not None]
            if valid_averages:
                full_avg = np.mean(np.array(valid_averages), axis=0)
                rospy.loginfo("\n--- Average of Full System (All Views) ---")
                rospy.loginfo(f"  Average of Full System: x={full_avg[0]:.4f}, y={full_avg[1]:.4f}, z={full_avg[2]:.4f}")
            else:
                rospy.logwarn("  No valid average positions found from any viewpoint.")



if __name__ == '__main__':
    DopeDetectorMultiViewComparison()

