#!/usr/bin/env python3

import rclpy
import cv2
import mediapipe as mp
from rclpy.node import Node
from media_pipe_ros2_msg.msg import PosePoint, MediaPipeHumanPose, MediaPipeHumanPoseList
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
mp_drawing_styles = mp.solutions.drawing_styles

class PosePublisher(Node):

    def __init__(self):
        super().__init__('mediapipe_pose_publisher')
        
        # Publishers
        self.publisher_ = self.create_publisher(MediaPipeHumanPoseList, '/mediapipe/human_pose_list', 10)
        self.image_publisher_ = self.create_publisher(Image, '/mediapipe/pose/image', 10)
        self.raw_image_publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # CvBridge
        self.bridge = CvBridge()
        
        # Camera setup
        self.declare_parameter('camera_index', 0)
        camera_index = self.get_parameter('camera_index').value
        self.cap = cv2.VideoCapture(camera_index)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Nu pot deschide camera {camera_index}!')
            return
        
        self.get_logger().info(f'Camera {camera_index} deschisă cu succes!')
        
        # Timer pentru procesare continuă
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 FPS
        
        # MediaPipe Pose
        self.pose = mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,  # 0, 1, sau 2 (mai precis dar mai lent)
            smooth_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

    def process_frame(self):
        success, image = self.cap.read()
        if not success:
            self.get_logger().warn("Nu pot citi frame de la cameră")
            return
        
        # Publică imaginea raw
        raw_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.raw_image_publisher_.publish(raw_image_msg)
        
        # Procesare MediaPipe
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = self.pose.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        imageHeight, imageWidth, _ = image.shape
        
        # Creează message-uri
        pose_list = MediaPipeHumanPoseList()
        human_pose = MediaPipeHumanPose()
        
        if results.pose_landmarks:
            # Desenează landmarks
            mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style()
            )
            
            # Extrage landmarks (33 puncte)
            for idx, landmark in enumerate(results.pose_landmarks.landmark):
                pose_point = PosePoint()
                pose_point.name = mp_pose.PoseLandmark(idx).name
                pose_point.x = landmark.x
                pose_point.y = landmark.y
                pose_point.z = landmark.z
                pose_point.visibility = landmark.visibility
                human_pose.pose_landmarks.append(pose_point)
            
            # Calculează centrul corpului (între șolduri)
            left_hip = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP]
            right_hip = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP]
            
            human_pose.center_x = (left_hip.x + right_hip.x) / 2.0
            human_pose.center_y = (left_hip.y + right_hip.y) / 2.0
            
            # Calculează înălțimea corpului (nas la șolduri)
            nose = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]
            human_pose.body_height = abs(nose.y - human_pose.center_y)
            
            pose_list.num_humans = 1
            
            # Debug info
            self.get_logger().debug(f'Pose detected: center=({human_pose.center_x:.2f}, {human_pose.center_y:.2f}), '
                                   f'height={human_pose.body_height:.2f}')
        else:
            # Nu există persoană detectată
            pose_list.num_humans = 0
        
        pose_list.human_pose = human_pose
        self.publisher_.publish(pose_list)
        
        # Publică imaginea cu detecție
        detection_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_publisher_.publish(detection_image_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    
    try:
        rclpy.spin(pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        pose_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
