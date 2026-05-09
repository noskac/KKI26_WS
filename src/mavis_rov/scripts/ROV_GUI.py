#!/usr/bin/env python3
import rospy
import tkinter as tk
from PIL import Image, ImageTk
import cv2
import datetime
import numpy as np
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String, Int32MultiArray, Float32
from geometry_msgs.msg import Vector3

class ROVControlStation:
    def __init__(self, root):
        self.root = root
        self.root.title("ROV GCS - Dashboard")
        self.root.configure(bg="#1e1e1e")

        # Subscriber untuk MPU/IMU
        rospy.Subscriber('/rov/imu_euler', Vector3, self.imu_callback)
        rospy.Subscriber('/rov/depth', Float32, self.depth_callback)

        # Inisialisasi ROS Node & Bridge
        rospy.init_node('rov_gui_node', anonymous=True)

        # Variabel penampung data dari ROS
        self.current_frame = None
        self.current_qr_data = "Menunggu..."

        # ROS Subscribers (Mendengarkan topik dari QR_Scan.py)
        rospy.Subscriber('/rov/camera/image_raw', RosImage, self.image_callback)
        rospy.Subscriber('/rov/qr_data', String, self.qr_callback)

        # --- KONFIGURASI LAYOUT GRID ---
        for i in range(3):
            self.root.grid_columnconfigure(i, weight=1)

        # ==========================================
        # 1. TOP INFORMATION BAR
        # ==========================================
        self.top_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.top_frame.grid(row=0, column=0, columnspan=3, sticky="ew", padx=10, pady=5)

        self.team_label = tk.Label(self.top_frame, text="Team: MAIVS EVO | Univ: Universitas Negeri Yogyakarta", bg="#2d2d2d", fg="white", font=("Courier", 12, "bold"))
        self.team_label.pack(side="left", padx=10, pady=8)

        self.time_label = tk.Label(self.top_frame, text="Day, Date Time", bg="#2d2d2d", fg="#00ff00", font=("Courier", 12))
        self.time_label.pack(side="right", padx=10, pady=8)

        # ==========================================
        # 2. ROW 1: CAMERA & QR STATUS
        # ==========================================
        self.cam1_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.cam1_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        tk.Label(self.cam1_frame, text="CAMERA 1\n(Front Cam)", bg="#2d2d2d", fg="white", font=("Courier", 10)).pack(pady=5)
        self.cam1_label = tk.Label(self.cam1_frame, bg="black", width=40, height=15)
        self.cam1_label.pack(padx=10, pady=10, expand=True)

        self.cam2_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.cam2_frame.grid(row=1, column=1, sticky="nsew", padx=10, pady=5)
        tk.Label(self.cam2_frame, text="CAMERA 2\n(Bottom / Side Cam)", bg="#2d2d2d", fg="white", font=("Courier", 10)).pack(pady=5)
        self.cam2_label = tk.Label(self.cam2_frame, bg="#111111", text="[ DISABLED - 1 CAM MODE ]", fg="gray", width=35, height=12)
        self.cam2_label.pack(padx=10, pady=10, expand=True)

        self.qr_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.qr_frame.grid(row=1, column=2, sticky="nsew", padx=10, pady=5)
        tk.Label(self.qr_frame, text="QR CODE & STATUS", bg="#2d2d2d", fg="white", font=("Courier", 12, "bold")).pack(pady=15)
        
        self.qr_side_label = tk.Label(self.qr_frame, text="- Side A/B/C/D : Menunggu...", bg="#2d2d2d", fg="yellow", font=("Courier", 11))
        self.qr_side_label.pack(anchor="w", padx=20, pady=10)
        
        self.qr_valid_label = tk.Label(self.qr_frame, text="- Status       : Invalid", bg="#2d2d2d", fg="red", font=("Courier", 11))
        self.qr_valid_label.pack(anchor="w", padx=20, pady=10)

        # ==========================================
        # -- ROV ORIENTATION / IMU (Kolom 2, Bawah QR) --
        # ==========================================
        self.imu_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.imu_frame.grid(row=2, column=2, sticky="nsew", padx=10, pady=5)
        
        tk.Label(self.imu_frame, text="ROV ORIENTATION (IMU)", bg="#2d2d2d", fg="white", font=("Courier", 12, "bold")).pack(pady=10)
        
        # Penampung Data (Bisa diupdate via ROS)
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_yaw = 0.0

        #Penampung Data (Bisa diupdate via ROS)
        self.current_depth = 0.0

        # Label Pitch (Warna Hijau)
        self.pitch_label = tk.Label(self.imu_frame, text="PITCH :  0.0°", bg="#2d2d2d", fg="#00ff00", font=("Courier", 16, "bold"))
        self.pitch_label.pack(pady=5)

        # Label Roll (Warna Cyan)
        self.roll_label = tk.Label(self.imu_frame, text="ROLL  :  0.0°", bg="#2d2d2d", fg="#00ffff", font=("Courier", 16, "bold"))
        self.roll_label.pack(pady=5)

        # Label Yaw (Warna Kuning)
        self.yaw_label = tk.Label(self.imu_frame, text="YAW   :  0.0°", bg="#2d2d2d", fg="#ffff00", font=("Courier", 16, "bold"))
        self.yaw_label.pack(pady=5)

        # ==========================================
        # 3. ROW 2: TELEMETRY & DESIGN (Disederhanakan untuk test)
        # ==========================================
        self.alt_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.alt_frame.grid(row=2, column=0, sticky="nsew", padx=10, pady=5)
        tk.Label(self.alt_frame, text="ALTITUDE\n(Height from pool floor)", bg="#2d2d2d", fg="white", font=("Courier", 10)).pack(pady=10)
        self.depth_label = tk.Label(self.alt_frame, text="0.00 m", bg="#2d2d2d", fg="cyan", font=("Courier", 30, "bold"))
        self.depth_label.pack(pady=20)

        # ==========================================
        # 4. FOOTER STATUS BAR
        # ==========================================
        self.footer_frame = tk.Frame(root, bg="#1e1e1e", bd=1, relief="ridge")
        self.footer_frame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=10, pady=5)
        status_text = " Mode: MANUAL (Remote)   |   Connection: ONLINE   |   Sensor Status: OK   |   Logging: STANDBY "
        tk.Label(self.footer_frame, text=status_text, bg="#1e1e1e", fg="orange", font=("Courier", 10, "bold")).pack(pady=8)

        # Mulai loop Tkinter
        self.update_gui()

    # Callback ketika ada gambar baru dari ROS
    def image_callback(self, msg):
        try:
            # Bypass cv_bridge: Konversi manual dari bytes ROS ke array Numpy OpenCV
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            self.current_frame = cv_image
        except Exception as e:
            print("Error frame callback:", e)
    
    # Callback ketika ada data QR baru dari ROS
    def qr_callback(self, msg):
        self.current_qr_data = msg.data

    def imu_callback(self, msg):
        self.current_pitch = msg.x
        self.current_roll = msg.y
        self.current_yaw = msg.z

    def depth_callback(self, msg):
        self.current_depth = msg.data

    def update_gui(self):
        # Update Waktu
        now = datetime.datetime.now().strftime("%A, %d-%m-%Y %H:%M:%S")
        self.time_label.config(text=now)

        # Update Data IMU
        self.pitch_label.config(text=f"PITCH : {self.current_pitch:>5.1f}°")
        self.roll_label.config(text=f"ROLL  : {self.current_roll:>5.1f}°")
        self.yaw_label.config(text=f"YAW   : {self.current_yaw:>5.1f}°")
        
        # Update Data depth
        self.depth_label.config(text=f"{self.current_depth:.2f} m")

        # Update Frame Kamera
        if self.current_frame is not None:
            f1_rgb = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
            img1 = ImageTk.PhotoImage(image=Image.fromarray(f1_rgb))
            self.cam1_label.imgtk = img1
            self.cam1_label.configure(image=img1, width=320, height=240)

        # Update Teks QR
        if self.current_qr_data != "Belum terdeteksi":
            self.qr_side_label.config(text=f"- Data QR : {self.current_qr_data}", fg="white")
            self.qr_valid_label.config(text="- Status  : Valid", fg="#00ff00")
        else:
            self.qr_side_label.config(text="- Data QR : Menunggu...", fg="yellow")
            self.qr_valid_label.config(text="- Status  : Invalid", fg="red")

        # Looping setiap 30ms
        self.root.after(30, self.update_gui)

if __name__ == "__main__":
    root = tk.Tk()
    app = ROVControlStation(root)
    root.mainloop()