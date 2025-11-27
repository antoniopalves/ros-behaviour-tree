#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ConeDetector(Node):
    def __init__(self):
        super().__init__('cone_detector')
        print("ConeDetector LIGADO")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/intel_realsense_r200_depth/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(Bool, '/cone_detected', 10)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print("Erro na conversão:", e)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # MÁSCARA CINZENTA (white-ish do cone)
        lower_gray = np.array([0, 0, 120])
        upper_gray = np.array([180, 60, 230])
        mask_gray = cv2.inRange(hsv, lower_gray, upper_gray)

        # MÁSCARA LARANJA (base do cone)
        lower_orange = np.array([5, 70, 70])
        upper_orange = np.array([25, 255, 255])
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # COMBINADO
        combined = cv2.bitwise_or(mask_gray, mask_orange)

        detected = False

        # ENCONTRAR ÁREAS LARANJA
        contours_orange, _ = cv2.findContours(
            mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours_orange:
            area = cv2.contourArea(c)
            if area < 200:
                continue

            x, y, w, h = cv2.boundingRect(c)

            # PROCURAR CINZENTO EM CIMA DO LARANJA
            y_top = max(y - int(h * 0.8), 0)
            gray_region = mask_gray[y_top:y, x:x+w]

            if gray_region.size == 0:
                continue

            if cv2.countNonZero(gray_region) > 80:
                detected = True
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
                break  # basta um cone

        # PUBLICA APENAS O BOOLEAN
        self.publisher.publish(Bool(data=detected))

        if detected:
            print("Cone DETETADO")

        # DEBUG VISUAL
        cv2.imshow("View", frame)
        cv2.imshow("Mask Gray", mask_gray)
        cv2.imshow("Mask Orange", mask_orange)
        cv2.imshow("Combined", combined)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
