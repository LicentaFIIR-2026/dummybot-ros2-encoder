import rclpy
import cv2
import mediapipe as mp
from rclpy.node import Node
from media_pipe_ros2_msg.msg import HandPoint, MediaPipeHumanHand, MediaPipeHumanHandList
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

class HandsPublisher(Node):

    def __init__(self):
        super().__init__('mediapipe_hands_publisher')
        
        # Publishers
        self.publisher_ = self.create_publisher(MediaPipeHumanHandList, '/mediapipe/human_hand_list', 10)
        self.image_publisher_ = self.create_publisher(Image, '/mediapipe/hands/image', 10)
        self.raw_image_publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # CvBridge
        self.bridge = CvBridge()
        
        # Camera setup - configurabil
        self.declare_parameter('camera_index', 0)
        camera_index = self.get_parameter('camera_index').value
        self.cap = cv2.VideoCapture(camera_index)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Nu pot deschide camera {camera_index}!')
            return
        
        self.get_logger().info(f'Camera {camera_index} deschisă cu succes!')
        
        # Timer pentru procesare continuă
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 FPS
        
        # MediaPipe Hands
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=2
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
        results = self.hands.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        # Creează message-uri
        mediapipehumanlist = MediaPipeHumanHandList()
        mediapipehuman = MediaPipeHumanHand()
        
        # Inițializează cu 0
        for i in range(21):
            mediapipehuman.right_hand_key_points.append(HandPoint(name=str(i), x=0.0, y=0.0, z=0.0))
            mediapipehuman.left_hand_key_points.append(HandPoint(name=str(i), x=0.0, y=0.0, z=0.0))
        
        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                
                # Desenează landmarks
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                hand_label = handedness.classification[0].label
                
                for idx, landmark in enumerate(hand_landmarks.landmark):
                    if hand_label == "Right":
                        mediapipehuman.right_hand_key_points[idx].name = str(idx)
                        mediapipehuman.right_hand_key_points[idx].x = landmark.x
                        mediapipehuman.right_hand_key_points[idx].y = landmark.y
                        mediapipehuman.right_hand_key_points[idx].z = landmark.z
                    elif hand_label == "Left":
                        mediapipehuman.left_hand_key_points[idx].name = str(idx)
                        mediapipehuman.left_hand_key_points[idx].x = landmark.x
                        mediapipehuman.left_hand_key_points[idx].y = landmark.y
                        mediapipehuman.left_hand_key_points[idx].z = landmark.z
        
        # Publică rezultatele
        mediapipehumanlist.human_hand_list = mediapipehuman
        mediapipehumanlist.num_humans = 1
        self.publisher_.publish(mediapipehumanlist)
        
        # Publică imaginea cu detecție
        detection_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_publisher_.publish(detection_image_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    hands_publisher = HandsPublisher()
    
    try:
        rclpy.spin(hands_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        hands_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
