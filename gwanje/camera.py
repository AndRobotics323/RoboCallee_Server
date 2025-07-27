
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import json

from scipy.spatial.transform import Rotation as R_


import threading
import time


'''
<아르코 마커의 pose: x, y, yaw>
message 타입: geometry_msgs.msg의 PoseStamped
토픽명: /aruco_pose{id}
<아르코 마커의 지나간 경로>
message 타입: nav_msgs.msg의 Path
토픽명: /aruco_path{id}
'''


class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        # self.pose1_publisher = self.create_publisher(PoseStamped, '/aruco_pose1', 10)
        # self.pose2_publisher = self.create_publisher(PoseStamped, '/aruco_pose2', 10)
        # self.pose3_publisher = self.create_publisher(PoseStamped, '/aruco_pose3', 10)


        self.pose_publishers = {
            1: self.create_publisher(PoseStamped, '/aruco_pose1', 10),
            2: self.create_publisher(PoseStamped, '/aruco_pose2', 10),
            3: self.create_publisher(PoseStamped, '/aruco_pose3', 10),
        }





    def publish_markers(self, marker_list):
        msg = String()
        msg.data = json.dumps(marker_list)
        self.publisher.publish(msg)














class VideoCamera:
    def __init__(self, dev_num=0):
        #카메라 오픈
        self.cap = cv2.VideoCapture(dev_num)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


        base_path = '/home/addinedu/djangos/mysite/gwanje/' 
         #  카메라 내부 파라미터
        self.camera_matrix = np.load( base_path + 'camera_matrix.npy')
        self.dist_coeffs = np.load( base_path + 'dist_coeffs.npy')
        self.R = np.load( base_path + 'R_cam_to_table.npy')
        self.T = np.load( base_path + 'T_cam_to_table.npy')

        #  ArUco 세팅
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)

        #  마커 3D 좌표
        self.marker_size = 50
        self.marker_3d_edges = np.array([
            [0, 0, 0],
            [self.marker_size, 0, 0],
            [self.marker_size, self.marker_size, 0],
            [0, self.marker_size, 0],
        ], dtype='float64').reshape((4, 1, 3))



        # Frame storage
        self.frame = None
        self.latest_markers = []
        self.lock = threading.Lock()


        self.cur_poses = {}  # 현재 인식된 마커의 pose 저장



        # ROS2
        try:
            rclpy.init(args=None)
        except :
            print("ROS2 already initialized or not available.")
        
        self.ros_node = MarkerPublisher()



        # Threads
        self.running = True
        threading.Thread(target=self.update, daemon=True).start()
        threading.Thread(target=self.spin_ros, daemon=True).start()



        # self.lock = threading.Lock()
        # self.frame = None
        # threading.Thread(target=self.update, daemon=True).start()

    def spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)




    def update(self):

        # while True:
        while self.running:
            ret, frame = self.cap.read()

            if not ret:
                continue

            # --- 5. 왜곡 제거 ---
            frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

            # --- 6. ArUco 검출 ---
            corners, ids, _ = self.detector.detectMarkers(frame_undistorted)


            # new_markers = []

            if ids is not None:
                for i, corner in enumerate(corners):

                    marker_id = int(ids[i][0])

                    corner = np.array(corner).reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corner


                    # 코너 표시
                    for pt in [topLeft, topRight, bottomRight, bottomLeft]:
                        cv2.circle(frame_undistorted, (int(pt[0]), int(pt[1])), 4, (255,0,0), -1)


                    # PnP
                    ret, rvec, tvec = cv2.solvePnP(
                        self.marker_3d_edges, 
                        corner, 
                        self.camera_matrix, 
                        self.dist_coeffs
                    )


                    if ret:

                        # (1) 카메라좌표계 위치,  # (2) 테이블좌표계 변환
                        trans = tvec.reshape(3, 1)  # (3,1)
                        # trans_applied = R_normalized @ trans + T_affine
                        trans_applied = self.R @ trans + self.T

                        x = round(trans_applied[0][0] / 1000 , 2) 
                        y = round(trans_applied[1][0] / 1000, 2) 
                        z = round(trans_applied[2][0] / 1000, 2) 


                        # 회전 변환 (회전 벡터 -> 행렬 -> 테이블 좌표계 적용)
                        rot_mat_cam, _ = cv2.Rodrigues(rvec)
                        # rot_mat_table = R_normalized @ rot_mat_cam
                        rot_mat_table = self.R @ rot_mat_cam
                    
                        # Euler 각도 radian (xyz 순서)
                        # rot_obj = R_.from_matrix(rot_mat_table)
                        # rx, ry, rz = rot_obj.as_euler('xyz', degrees=False)
                    
                        # rx = round(rx, 4)
                        # ry = round(ry, 4)
                        # rz = round(rz, 4)

                        # rot = R_.from_euler('xyz', [rx, ry, rz])
                        # q = rot.as_quat()


                        q = R_.from_matrix(rot_mat_table)

                        pose = PoseStamped()
                        # pose.header.stamp = self.get_clock().now().to_msg() 여기서 self는 node
                        pose.header.frame_id = "map"

                        pose.pose.position.x = float( x )
                        pose.pose.position.y = float( y )
                        pose.pose.position.z = float( 0 )

                        pose.pose.orientation.x = float( q[0] )
                        pose.pose.orientation.y = float( q[1] )
                        pose.pose.orientation.z = float( q[2] )
                        pose.pose.orientation.w = float( q[3] )


                        # new_markers.append({
                        #     "id": marker_id,
                        #     "x": x,
                        #     "y": y,
                        #     "rz": rz,
                        # })
                        
                        # 위치 및 회전 정보 표시
                        trans_text = f"Trans: ({x}, {y})m"
                        rot_text = f"Rot: ( {rz} )rad"

                        
                        # (4) 그리기
                        cv2.putText(frame_undistorted,
                                    trans_text,
                                    (int(topLeft[0]-150), int(topLeft[1]+10)),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 255, 0), 2)


                        cv2.putText(frame_undistorted,
                                    rot_text,
                                    (int(topLeft[0]-150), int(topLeft[1]+50)),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 255, 0), 2)



                        # cv2.drawFrameAxes(frame_undistorted, camera_matrix, dist_coeffs,
                                        # rvec, tvec, marker_size/2)


                        # with self.lock:
                        self.cur_poses[marker_id] = pose




            # Pose 값 publish
            for marker_id, pose in self.cur_poses.items():
                if marker_id in self.ros_node.pose_publishers:
                    self.ros_node.pose_publishers[marker_id].publish(pose)


            # Path 값 publish






            # Update latest markers
            # with self.lock:
                # self.latest_markers = new_markers

            # Publish to ROS2
            # self.ros_node.publish_markers(new_markers)


             # JPEG 인코딩
            ret, jpeg = cv2.imencode('.jpg', frame_undistorted)
            if ret:
                with self.lock:
                    self.frame = jpeg.tobytes()





    def get_frame(self):
        with self.lock:
            return self.frame
        


    def get_markers(self):
        with self.lock:
            return self.latest_markers



    def stop(self):
        self.running = False
        self.cap.release()
        cv2.destroyAllWindows()



    def __del__(self):
        self.cap.release()