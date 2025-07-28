#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import json
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import threading



def yaw_to_quaternion(yaw):
    """Z축 회전(yaw, radian)을 쿼터니언(x, y, z, w)으로 변환."""
    q = {}
    q['x'] = 0.0
    q['y'] = 0.0
    q['z'] = math.sin(yaw / 2.0)
    q['w'] = math.cos(yaw / 2.0)
    return q



class ArucoDetector:
    def __init__(self, camera_matrix, dist_coeffs, marker_length=0.05):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_length = marker_length

        self.src_pts = np.array([
            [-0.96,  0.41],
            [-0.98, -0.49],
            [ 0.97,  0.39],
            [ 0.99, -0.53]
        ], dtype=np.float32)
        self.dst_pts = np.array([
            [0.05, 0.05],
            [0.05, 0.95],
            [1.95, 0.05],
            [1.95, 0.95]
        ], dtype=np.float32)
        self.H = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)

    def transform_point(self, pt):
        src = np.array([pt[0], pt[1], 1.0], dtype=np.float32)
        dst = self.H @ src
        dst = dst / dst[2]
        return float(dst[0]), float(dst[1])


    def detect_markers(self, frame):
        corners, ids, _ = self.detector.detectMarkers(frame)
        detected = []
        if ids is not None:
            half_len = self.marker_length / 2.0

            # 마커의 3D 실제 좌표 (Z=0에 평면, 정사각형)
            object_points = np.array([
                [-half_len,  half_len, 0],
                [-half_len, -half_len, 0],
                [ half_len, -half_len, 0],
                [ half_len,  half_len, 0]
            ], dtype=np.float32)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                image_points = corners[i][0].astype(np.float32)  # 4x2

                success, rvec, tvec = cv2.solvePnP(
                    object_points, image_points,
                    self.camera_matrix, self.dist_coeffs
                )

                if not success:
                    continue

                tvec = tvec.reshape(-1)
                rvec = rvec.reshape(-1)
                x, y = self.transform_point((tvec[0], tvec[1]))
                x, y = round(x, 2), round(y, 2)

                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw = np.arctan2(rot_mat[1, 0], rot_mat[0, 0]) - np.pi / 2
                yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
                yaw = round(float(yaw), 3)

                corners_reshaped = corners[i].reshape((4, 2))
                center_x = int(np.mean(corners_reshaped[:, 0]))
                center_y = int(np.mean(corners_reshaped[:, 1]))

                detected.append({
                    "id": marker_id,
                    "robot_xy": [x, y],
                    "yaw": yaw,
                    "text_pos": (center_x, center_y)
                })
        return detected

'''
    def detect_markers(self, frame):
        corners, ids, _ = self.detector.detectMarkers(frame )
        detected = []
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers( 
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]
                x, y = self.transform_point((tvec[0], tvec[1]))
                x, y = round(x, 2), round(y, 2)

                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw = np.arctan2(rot_mat[1, 0], rot_mat[0, 0]) - np.pi / 2
                yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
                yaw = round(float(yaw), 3)

                corners_reshaped = corners[i].reshape((4, 2))
                center_x = int(np.mean(corners_reshaped[:, 0]))
                center_y = int(np.mean(corners_reshaped[:, 1]))

                detected.append({
                    "id": marker_id,
                    "robot_xy": [x, y],
                    "yaw": yaw,
                    "text_pos": (center_x, center_y)
                })
        return detected

'''


class VideoCamera(Node):
    def __init__(self, dev_num, camera_matrix, dist_coeffs):
        super().__init__('aruco_path_tracker_id')
        self.detector = ArucoDetector(camera_matrix, dist_coeffs)
        self.cap = cv2.VideoCapture(dev_num)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        self.json_pub = self.create_publisher(String, 'detected_markers', 10)
        self.path_publishers = {}  # id -> publisher for Path
        self.pose_publishers = {}  # id -> publisher for PoseStamped
        self.paths = {}            # id -> Path()
        self.tf_broadcaster = TransformBroadcaster(self)


        self.frame = None

        self.lock = threading.Lock()




    def run(self):

        try:
            while rclpy.ok():
                self.process_frame()
                rclpy.spin_once(self, timeout_sec=0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()
            rclpy.shutdown()





    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame not received!")
            return

        detected = self.detector.detect_markers(frame)
        self.json_pub.publish(String(data=json.dumps([
            {"id": d["id"], "robot_xy": d["robot_xy"], "yaw": d["yaw"]} for d in detected
        ])))

        for marker in detected:
            marker_id = marker["id"]
            x, y = marker["robot_xy"]
            yaw = marker["yaw"]
            cx, cy = marker["text_pos"]

            # 화면 표시
            cv2.putText(frame, f"ID: {marker_id}", (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 2)
            cv2.putText(frame, f"XY: ({x:.2f}, {y:.2f})", (cx + 10, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 2)
            cv2.putText(frame, f"Yaw: {yaw:.2f}", (cx + 10, cy + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 2)

            # Path publisher
            path_topic = f"/aruco_path{marker_id}"
            if marker_id not in self.path_publishers:
                self.path_publishers[marker_id] = self.create_publisher(Path, path_topic, 10)
                path = Path()
                path.header.frame_id = "map"
                self.paths[marker_id] = path

            # PoseStamped publisher
            pose_topic = f"/aruco_pose{marker_id}"
            if marker_id not in self.pose_publishers:
                self.pose_publishers[marker_id] = self.create_publisher(PoseStamped, pose_topic, 10)

            # 현재 pose
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # 쿼터니언 적용
            q = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = q['x']
            pose.pose.orientation.y = q['y']
            pose.pose.orientation.z = q['z']
            pose.pose.orientation.w = q['w']

            # Path 누적 + publish
            self.paths[marker_id].header.stamp = self.get_clock().now().to_msg()
            self.paths[marker_id].poses.append(pose)
            self.path_publishers[marker_id].publish(self.paths[marker_id])

            # PoseStamped publish
            self.pose_publishers[marker_id].publish(pose)
            
            # TF 브로드캐스트
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"  # 부모 프레임
            t.child_frame_id = f"aruco_{marker_id}"  # 자식 프레임

            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q['x']
            t.transform.rotation.y = q['y']
            t.transform.rotation.z = q['z']
            t.transform.rotation.w = q['w']

            self.tf_broadcaster.sendTransform(t)

        # cv2.imshow("Aruco Path Tracker by ID", frame)
        # with self.lock:
        self.frame = frame
        
        # cv2.waitKey(1)



    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()



    def get_frame(self):
        with self.lock:
            if self.frame is None:
                return None
            ret, jpeg = cv2.imencode('.jpg', self.frame)
            if not ret:
                return None
            return jpeg.tobytes()  # 🔥 바이트로 반환!
        




# def main():
#     rclpy.init()
#     base_path = '/home/addinedu/djangos/RoboCallee_Server/gwanje/'
#     camera_matrix = np.load(base_path + 'camera_matrix.npy')
#     dist_coeffs = np.load(base_path + 'dist_coeffs.npy')

#     node = VideoCamera(camera_matrix, dist_coeffs)
#     try:
#         while rclpy.ok():
#             node.process_frame()
#             rclpy.spin_once(node, timeout_sec=0.01)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()








