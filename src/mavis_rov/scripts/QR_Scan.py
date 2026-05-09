#!/usr/bin/env python3
import rospy
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import threading
import time

# Import ROS Message Types
from std_msgs.msg import String
from sensor_msgs.msg import Image

class VisionSystem:
    # --- KODE ANDA TETAP SAMA SEPERTI SEBELUMNYA ---
    def __init__(self, cam1_index=0):
        self.cap1 = cv2.VideoCapture(cam1_index)
        self.setup_camera(self.cap1)
        self.last_qr_data = "Belum terdeteksi"
        self.last_qr_pts = None 
        self.latest_frame = None
        self.ret = False
        self.running = True
        self.prev_time = 0
        self.fps = 0
        self.target_fps = 30  
        self.thread = threading.Thread(target=self._update_camera_loop, daemon=True)
        self.thread.start()

    def setup_camera(self, cap):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

    def _update_camera_loop(self):
        frame_counter = 0
        target_frame_time = 1.0 / self.target_fps
        while self.running:
            loop_start_time = time.time()
            ret, frame = self.cap1.read()
            if ret:
                current_time = time.time()
                self.fps = 1.0 / (current_time - self.prev_time) if self.prev_time > 0 else 0
                self.prev_time = current_time

                if frame_counter % 5 == 0:
                    self._scan_qr(frame)
                frame_counter += 1

                if self.last_qr_pts is not None:
                    cv2.polylines(frame, [self.last_qr_pts], True, (0, 255, 0), 2)

                cv2.putText(frame, f"FPS: {int(self.fps)}", (15, 35), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                self.latest_frame = frame
                self.ret = True
            else:
                self.ret = False
                
            elapsed_time = time.time() - loop_start_time
            sleep_time = target_frame_time - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                time.sleep(0.001)

    def _scan_qr(self, frame):
        # 1. Konversi frame ke Grayscale agar pyzbar lebih mudah membaca
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 2. Decode dari gambar grayscale
        decoded_objects = decode(gray)
        
        if decoded_objects:
            for obj in decoded_objects:
                self.last_qr_data = obj.data.decode('utf-8')
                pts = obj.polygon
                if len(pts) == 4:
                    self.last_qr_pts = np.array(pts, np.int32).reshape((-1, 1, 2))
        else:
            # 3. Reset data jika QR Code sudah tidak ada di depan kamera
            self.last_qr_pts = None
            self.last_qr_data = "Belum terdeteksi"

    def get_frames(self):
        if self.ret and self.latest_frame is not None:
            return self.ret, self.latest_frame.copy(), self.last_qr_data
        return False, None, self.last_qr_data

    def release(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        self.cap1.release()

# --- BAGIAN ROS NODE ---# --- FUNGSI BYPASS CV_BRIDGE ---
def convert_cv_to_ros_msg(cv_image):
    """Konversi manual dari OpenCV/Numpy ke ROS sensor_msgs/Image"""
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.step = cv_image.shape[1] * 3  # 3 channel (B, G, R)
    img_msg.data = cv_image.tobytes()
    return img_msg

# --- BAGIAN ROS NODE ---
def main():
    rospy.init_node('qr_scanner_node', anonymous=True)
    
    image_pub = rospy.Publisher('/rov/camera/image_raw', Image, queue_size=1)
    qr_pub = rospy.Publisher('/rov/qr_data', String, queue_size=10)
    
    vision = VisionSystem(cam1_index=0)
    rate = rospy.Rate(30)
    
    rospy.loginfo("Node QR Scanner menyala. Mengakses kamera...")

    while not rospy.is_shutdown():
        ret, frame, qr_data = vision.get_frames()
        
        if ret and frame is not None:
            try:
                # Menggunakan fungsi manual, BUKAN CvBridge
                img_msg = convert_cv_to_ros_msg(frame)
                image_pub.publish(img_msg)
            except Exception as e:
                rospy.logerr("Gagal konversi gambar: %s", str(e))
            
            qr_pub.publish(qr_data)
            
        rate.sleep()

    vision.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass