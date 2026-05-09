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
        self.root.title("ROV GCS - Dashboard (Responsive - ROS Edition)")
        self.root.configure(bg="#1e1e1e")

        # Inisialisasi ROS Node
        rospy.init_node('rov_gui_node', anonymous=True)

        # ==========================================
        # VARIABEL PENAMPUNG DATA (Mulai dari 0 / Default)
        # ==========================================
        self.current_frame = None
        self.current_qr_data = "Menunggu..."
        
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_yaw = 0.0
        self.current_depth = 0.0
        
        # Default PWM idle adalah 1500
        self.current_pwm = {
            "DKIRI": 1500, "TKIRI": 1500, "BKIRI": 1500,
            "DKANAN": 1500, "TKANAN": 1500, "BKANAN": 1500
        }

        # ==========================================
        # ROS SUBSCRIBERS
        # ==========================================
        rospy.Subscriber('/rov/camera/image_raw', RosImage, self.image_callback)
        rospy.Subscriber('/rov/qr_data', String, self.qr_callback)
        rospy.Subscriber('/rov/imu_euler', Vector3, self.imu_callback)
        rospy.Subscriber('/rov/depth', Float32, self.depth_callback)
        # Tambahan: Subscriber untuk baca nilai PWM thruster (Array isi 6 angka)
        rospy.Subscriber('/rov/thruster_pwm', Int32MultiArray, self.pwm_callback)

        # --- KONFIGURASI LAYOUT GRID (Responsif) ---
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_columnconfigure(2, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, weight=1)

        # ==========================================
        # 1. TOP INFORMATION BAR
        # ==========================================
        self.top_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.top_frame.grid(row=0, column=0, columnspan=3, sticky="ew", padx=10, pady=5)

        self.team_label = tk.Label(self.top_frame, text="Team: MAIVS EVO | Univ: Universitas Negeri Yogyakarta", bg="#2d2d2d", fg="white", font=("Courier", 12, "bold"))
        self.team_label.pack(side="left", padx=10, pady=5)

        self.time_label = tk.Label(self.top_frame, text="Day, Date Time", bg="#2d2d2d", fg="#00ff00", font=("Courier", 12, "bold"))
        self.time_label.pack(side="right", padx=10, pady=5)

        # ==========================================
        # 2. ROW 1: CAMERA & QR STATUS
        # ==========================================
        self.cam1_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.cam1_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        tk.Label(self.cam1_frame, text="CAMERA 1 (Front Cam)", bg="#2d2d2d", fg="white", font=("Courier", 12, "bold")).pack(pady=5)
        self.cam1_label = tk.Label(self.cam1_frame, bg="black")
        self.cam1_label.pack(padx=10, pady=10, expand=True, fill="both")

        self.cam2_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.cam2_frame.grid(row=1, column=1, sticky="nsew", padx=10, pady=5)
        tk.Label(self.cam2_frame, text="CAMERA 2 (Bottom / Side Cam)", bg="#2d2d2d", fg="white", font=("Courier", 12, "bold")).pack(pady=5)
        self.cam2_label = tk.Label(self.cam2_frame, bg="#111111", text="[ DISABLED - 1 CAM MODE ]", fg="gray", font=("Courier", 12))
        self.cam2_label.pack(padx=10, pady=10, expand=True, fill="both")

        self.qr_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.qr_frame.grid(row=1, column=2, sticky="nsew", padx=10, pady=5)
        tk.Label(self.qr_frame, text="QR CODE & STATUS", bg="#2d2d2d", fg="white", font=("Courier", 14, "bold")).pack(pady=20)
        
        self.qr_side_label = tk.Label(self.qr_frame, text="- Data QR : Menunggu...", bg="#2d2d2d", fg="yellow", font=("Courier", 12))
        self.qr_side_label.pack(anchor="w", padx=20, pady=10)
        self.qr_valid_label = tk.Label(self.qr_frame, text="- Status  : Invalid", bg="#2d2d2d", fg="red", font=("Courier", 12))
        self.qr_valid_label.pack(anchor="w", padx=20, pady=10)

        # ==========================================
        # 3. ROW 2: TELEMETRY (Bawah)
        # ==========================================
        
        # --- KOLOM KIRI (Altitude + Thruster) ---
        self.col0_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.col0_frame.grid(row=2, column=0, sticky="nsew", padx=10, pady=5)
        
        tk.Label(self.col0_frame, text="ALTITUDE (Height from pool floor)", bg="#2d2d2d", fg="white", font=("Courier", 10)).pack(pady=5)
        self.depth_label = tk.Label(self.col0_frame, text="0.00 m", bg="#2d2d2d", fg="cyan", font=("Courier", 30, "bold"))
        self.depth_label.pack(pady=5)

        tk.Frame(self.col0_frame, height=2, bg="#555555").pack(fill="x", padx=20, pady=5)

        tk.Label(self.col0_frame, text="ROV THRUSTER (PWM)", bg="#2d2d2d", fg="white", font=("Courier", 12, "bold")).pack(pady=5)
        
        self.t_canvas = tk.Canvas(self.col0_frame, width=300, height=200, bg="#111111", highlightthickness=1, highlightbackground="#555555")
        self.t_canvas.pack(pady=5)

        self.t_canvas.create_rectangle(100, 30, 200, 170, outline="#00aaff", width=2, fill="#1a2b3c")
        self.t_texts = {}

        def draw_thruster(tag, x, y, is_vertical=True):
            if is_vertical:
                self.t_canvas.create_oval(x-22, y-22, x+22, y+22, outline="#888888", width=1, fill="#222222")
            else:
                self.t_canvas.create_oval(x-28, y-15, x+28, y+15, outline="#888888", width=1, fill="#222222")
            
            lbl = self.t_canvas.create_text(x, y, text=f"{tag}\n1500", fill="white", font=("Courier", 8, "bold"), justify="center")
            self.t_texts[tag] = lbl

        draw_thruster("DKIRI", 65, 45, is_vertical=True)
        draw_thruster("TKIRI", 60, 100, is_vertical=False)
        draw_thruster("BKIRI", 65, 155, is_vertical=True)
        draw_thruster("DKANAN", 235, 45, is_vertical=True)
        draw_thruster("TKANAN", 240, 100, is_vertical=False)
        draw_thruster("BKANAN", 235, 155, is_vertical=True)

        # --- KOLOM TENGAH (Trajectory Map) ---
        self.traj_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.traj_frame.grid(row=2, column=1, sticky="nsew", padx=10, pady=5)
        tk.Label(self.traj_frame, text="TRAJECTORY MAP", bg="#2d2d2d", fg="white", font=("Courier", 12, "bold")).pack(pady=5)
        
        self.traj_canvas = tk.Canvas(self.traj_frame, bg="black", highlightthickness=1, highlightbackground="#555555")
        self.traj_canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self.traj_canvas.create_text(20, 20, text="S", fill="green", font=("Courier", 14, "bold"))

        # --- KOLOM KANAN (IMU) ---
        self.imu_frame = tk.Frame(root, bg="#2d2d2d", bd=1, relief="ridge")
        self.imu_frame.grid(row=2, column=2, sticky="nsew", padx=10, pady=5)
        
        tk.Label(self.imu_frame, text="ROV ORIENTATION (IMU)", bg="#2d2d2d", fg="white", font=("Courier", 14, "bold")).pack(pady=15)

        self.pitch_label = tk.Label(self.imu_frame, text="PITCH :  0.0°", bg="#2d2d2d", fg="#00ff00", font=("Courier", 18, "bold"))
        self.pitch_label.pack(pady=10)
        self.roll_label = tk.Label(self.imu_frame, text="ROLL  :  0.0°", bg="#2d2d2d", fg="#00ffff", font=("Courier", 18, "bold"))
        self.roll_label.pack(pady=10)
        self.yaw_label = tk.Label(self.imu_frame, text="YAW   :  0.0°", bg="#2d2d2d", fg="#ffff00", font=("Courier", 18, "bold"))
        self.yaw_label.pack(pady=10)

        # ==========================================
        # 4. FOOTER STATUS BAR
        # ==========================================
        self.footer_frame = tk.Frame(root, bg="#1e1e1e", bd=1, relief="ridge")
        self.footer_frame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=10, pady=5)
        # Indikator diubah ke ONLINE untuk versi ROS
        status_text = " Mode: MANUAL (Remote)   |   Connection: ONLINE   |   Sensor Status: OK   |   Logging: STANDBY "
        tk.Label(self.footer_frame, text=status_text, bg="#1e1e1e", fg="orange", font=("Courier", 10, "bold")).pack(pady=8)

        self.update_gui()

    # ==========================================
    # FUNGSI CALLBACK ROS
    # ==========================================
    def image_callback(self, msg):
        try:
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            self.current_frame = cv_image
        except Exception as e:
            pass

    def qr_callback(self, msg):
        self.current_qr_data = msg.data

    def imu_callback(self, msg):
        self.current_pitch = msg.x
        self.current_roll = msg.y
        self.current_yaw = msg.z

    def depth_callback(self, msg):
        self.current_depth = msg.data

    def pwm_callback(self, msg):
        # Asumsi array dikirim urut: [DKIRI, TKIRI, BKIRI, DKANAN, TKANAN, BKANAN]
        if len(msg.data) >= 6:
            self.current_pwm["DKIRI"] = msg.data[0]
            self.current_pwm["TKIRI"] = msg.data[1]
            self.current_pwm["BKIRI"] = msg.data[2]
            self.current_pwm["DKANAN"] = msg.data[3]
            self.current_pwm["TKANAN"] = msg.data[4]
            self.current_pwm["BKANAN"] = msg.data[5]

    # ==========================================
    # LOOP UPDATE GUI
    # ==========================================
    def update_gui(self):
        # 1. Update Jam/Waktu
        now = datetime.datetime.now().strftime("%A, %d-%m-%Y %H:%M:%S")
        self.time_label.config(text=now)

        # 2. Update Frame Kamera dari ROS
        if self.current_frame is not None:
            try:
                target_w = self.cam1_label.winfo_width()
                target_h = self.cam1_label.winfo_height()
                
                if target_w > 10 and target_h > 10:
                    f1_resized = cv2.resize(self.current_frame, (target_w, target_h))
                else:
                    f1_resized = cv2.resize(self.current_frame, (320, 240))
                    
                f1_rgb = cv2.cvtColor(f1_resized, cv2.COLOR_BGR2RGB)
                img1 = ImageTk.PhotoImage(image=Image.fromarray(f1_rgb))
                self.cam1_label.imgtk = img1
                self.cam1_label.configure(image=img1)
            except Exception as e:
                pass

        # 3. Update Teks QR
        if self.current_qr_data != "Menunggu..." and self.current_qr_data != "Belum terdeteksi":
            self.qr_side_label.config(text=f"- Data QR : {self.current_qr_data}", fg="white")
            self.qr_valid_label.config(text="- Status  : Valid", fg="#00ff00")
        else:
            self.qr_side_label.config(text="- Data QR : Menunggu...", fg="yellow")
            self.qr_valid_label.config(text="- Status  : Invalid", fg="red")

        # 4. Update Data IMU & Depth Real-time (Tanpa Random)
        self.pitch_label.config(text=f"PITCH : {self.current_pitch:>5.1f}°")
        self.roll_label.config(text=f"ROLL  : {self.current_roll:>5.1f}°")
        self.yaw_label.config(text=f"YAW   : {self.current_yaw:>5.1f}°")
        self.depth_label.config(text=f"{self.current_depth:.2f} m")

        # 5. Update PWM Thruster Real-time
        for tag, text_id in self.t_texts.items():
            pwm_val = self.current_pwm[tag]
            # Cyan untuk maju/naik, merah untuk mundur/turun, putih jika idle
            color = "#00ffff" if pwm_val > 1510 else ("#ff4444" if pwm_val < 1490 else "white")
            self.t_canvas.itemconfig(text_id, text=f"{tag}\n{pwm_val}", fill=color)

        self.root.after(30, self.update_gui)

if __name__ == "__main__":
    root = tk.Tk()
    app = ROVControlStation(root)
    root.mainloop()