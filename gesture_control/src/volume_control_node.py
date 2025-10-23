import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import subprocess
from mediapipe.python import solutions as mp_solutions

class VolumeControlNode(Node):
    def __init__(self):
        super().__init__('volume_control_node')
        self.wcam, self.hcam = 640, 480
        self.cap = cv2.VideoCapture(2)
        self.cap.set(3, self.wcam)
        self.cap.set(4, self.hcam)
        self.detector = mp_solutions.hands.Hands()
        self.minVol = 0
        self.maxVol = 100  # Volume percentage for pactl
        self.get_logger().info('Volume control node initialized')
        
    def set_volume(self, vol_percent):
        """Set system volume using pactl (Linux)"""
        volume_cmd = f"pactl set-sink-volume @DEFAULT_SINK@ {int(vol_percent)}%"
        subprocess.run(volume_cmd, shell=True)
    
    def run(self):
        while True:
            success, img = self.cap.read()
            if not success:
                self.get_logger().error('Failed to capture image')
                break

            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = self.detector.process(img_rgb)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    index_finger = hand_landmarks.landmark[mp_solutions.hands.HandLandmark.INDEX_FINGER_TIP]
                    thumb = hand_landmarks.landmark[mp_solutions.hands.HandLandmark.THUMB_TIP]

                    # Calculate the distance between thumb and index finger
                    length = math.hypot(index_finger.x - thumb.x, index_finger.y - thumb.y)

                    # Map the length to the volume range
                    vol_percent = np.interp(length, [0, 0.2], [self.minVol, self.maxVol])
                    volBar = np.interp(length, [0, 0.2], [400, 150])
                    volPer = np.interp(length, [0, 0.2], [0, 100])

                    # Set volume using pactl
                    self.set_volume(volPer)

                    # Draw on the image
                    cv2.rectangle(img, (50, 150), (85, 400), (0, 255, 0), 3)
                    cv2.rectangle(img, (50, int(volBar)), (85, 400), (0, 255, 0), cv2.FILLED)
                    cv2.putText(img, f'{int(volPer)}%', (40, 450), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

                    # Draw circle when thumb and index are close
                    if length < 0.1:
                        cv2.circle(img, (int(thumb.x * self.wcam), int(thumb.y * self.hcam)), 15, (0, 255, 0), cv2.FILLED)

            cv2.imshow("Volume Control", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = VolumeControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down volume control node...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()