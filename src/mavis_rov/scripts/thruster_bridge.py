#!/usr/bin/env python3
import rospy
import serial
import threading
import time
import socket
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Int32MultiArray, Float32
from pynput import keyboard

class ThrusterBridge:
    def __init__(self):
        rospy.init_node('thruster_bridge_node', anonymous=True)
        
        # ================= KONEKSI SERIAL TEENSY =================
        port = rospy.get_param('~teensy_port', '/dev/ttyACM0')
        baud = rospy.get_param('~teensy_baudrate', 115200)
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            rospy.loginfo(f"[BRIDGE] Terhubung ke Teensy di {port}")
        except Exception as e:
            rospy.logerr(f"[BRIDGE] Gagal membuka serial {port}: {e}")
            self.ser = None

        # ================= ROS PUBLISHER =================
        self.imu_pub = rospy.Publisher('/rov/imu_euler', Vector3, queue_size=10)
        self.depth_pub = rospy.Publisher('/rov/depth', Float32, queue_size=10)
        self.mode_pub = rospy.Publisher('/rov/system_mode', String, queue_size=10)
        
        # Publisher BARU untuk mengirim data PWM ke GUI
        self.pwm_pub = rospy.Publisher('/rov/thruster_pwm', Int32MultiArray, queue_size=10)
        
        # ================= ROS SUBSCRIBER =================
        rospy.Subscriber('/rov/auto_cmd', Int32MultiArray, self.auto_ai_callback)

        # ================= STATE KENDALI =================
        self.mode = 1  # 1: Manual, 2: Auto AI
        
        # Array berisi 7 Elemen: [S, Y, H, R, T, TiltArm, Gripper]
        self.num_motions = 7
        self.target_motion = [1500, 1500, 1500, 1500, 1500, 0, 90]
        self.current_motion = [1500, 1500, 1500, 1500, 1500, 0, 90]
        
        self.ramp_step = 20
        self.ramp_delay = 1.0 / 90.0     
        self.running = True
        self.pressed_keys = set()
        
        # ================= KONFIGURASI UDP =================
        self.udp_port = 5005
        self.udp_last_recv_time = time.time()

        # ================= THREADING =================
        self.serial_thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.ramp_thread = threading.Thread(target=self.ramping_loop, daemon=True)
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.udp_thread = threading.Thread(target=self.udp_server_loop, daemon=True)
        
        self.serial_thread.start()
        self.ramp_thread.start()
        self.keyboard_thread.start()
        self.udp_thread.start()
        
        rospy.loginfo("[BRIDGE] Sistem siap! Mode: MANUAL.")

    # ================= LOGIKA MODE =================
    def set_mode(self, new_mode):
        if self.mode == new_mode: return
        self.mode = new_mode
        
        if self.mode == 1: 
            rospy.loginfo("[BRIDGE] MODE 1: MANUAL KONTROL")
        elif self.mode == 2: 
            rospy.loginfo("[BRIDGE] MODE 2: AUTO AI MENGAMBIL ALIH")
            # Auto-Brake (Hanya mereset 5 thruster baling-baling)
            self.target_motion[0:5] = [1500, 1500, 1500, 1500, 1500]

    def auto_ai_callback(self, msg):
        if self.mode == 2 and len(msg.data) >= 5:
            self.target_motion[0:5] = list(msg.data[0:5])

    # ================= UDP SERVER LOOP =================
    def udp_server_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.udp_port))
        sock.settimeout(0.5) 
        
        rospy.loginfo(f"[UDP] Menunggu data Gamepad di port {self.udp_port}...")
        
        while self.running and not rospy.is_shutdown():
            try:
                data, addr = sock.recvfrom(1024)
                self.udp_last_recv_time = time.time()
                
                vals = data.decode('utf-8').split(',')
                if len(vals) == 8: 
                    self.set_mode(int(vals[7]))
                    
                    if self.mode == 1 and not self.pressed_keys: 
                        self.target_motion = [int(v) for v in vals[:7]]
                        
            except socket.timeout:
                if self.mode == 1 and not self.pressed_keys and (time.time() - self.udp_last_recv_time) > 0.5:
                    # Failsafe Timeout: Hanya me-reset 5 baling-baling, servo menahan posisi
                    self.target_motion[0:5] = [1500, 1500, 1500, 1500, 1500]

    # ================= KEYBOARD FALLBACK =================
    def keyboard_listener(self):
        def on_press(key):
            try:
                if hasattr(key, 'char') and key.char in ['1', '2']:
                    self.set_mode(int(key.char))
                    return
                if self.mode != 1: return 
                if hasattr(key, 'char'):
                    self.pressed_keys.add(key.char.lower())
                    self.calculate_manual_keyboard()
            except AttributeError: pass

        def on_release(key):
            if self.mode != 1: return
            try:
                if hasattr(key, 'char'):
                    k = key.char.lower()
                    if k in self.pressed_keys: self.pressed_keys.remove(k)
                    self.calculate_manual_keyboard()
            except AttributeError: pass

        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    def calculate_manual_keyboard(self):
        s, y, h, r, t = 1500, 1500, 1500, 1500, 1500
        step = 400
        
        if 'w' in self.pressed_keys: s = 1500 + step
        elif 's' in self.pressed_keys: s = 1500 - step
        if 'd' in self.pressed_keys: y = 1500 + step
        elif 'a' in self.pressed_keys: y = 1500 - step
        if '7' in self.pressed_keys: h = 1500 + step
        elif '9' in self.pressed_keys: h = 1500 - step
        if '6' in self.pressed_keys: r = 1500 + step
        elif '4' in self.pressed_keys: r = 1500 - step
        if '8' in self.pressed_keys: t = 1500 + step
        elif '5' in self.pressed_keys: t = 1500 - step
        
        # 1. Update 5 Baling-baling
        self.target_motion[0:5] = [s, y, h, r, t]

        # 2. Update Gripper (Hold Position)
        if 'g' in self.pressed_keys: self.target_motion[6] = 180
        elif 'b' in self.pressed_keys: self.target_motion[6] = 0
        else: self.target_motion[6] = self.current_motion[6]

        # 3. Update Tilt Arm (Hold Position)
        if '+' in self.pressed_keys or '=' in self.pressed_keys: self.target_motion[5] = 180
        elif '-' in self.pressed_keys: self.target_motion[5] = 0
        else: self.target_motion[5] = self.current_motion[5]

    # ================= RAMPING & TRANSMISI =================
    def ramping_loop(self):
        while self.running and not rospy.is_shutdown():
            if self.mode == 1: self.mode_pub.publish("MANUAL (Remote)")
            elif self.mode == 2: self.mode_pub.publish("AUTO AI (Standby)")
            
            changed = False
            for i in range(self.num_motions):
                if self.current_motion[i] < self.target_motion[i]:
                    self.current_motion[i] = min(self.current_motion[i] + self.ramp_step, self.target_motion[i])
                    changed = True
                elif self.current_motion[i] > self.target_motion[i]:
                    self.current_motion[i] = max(self.current_motion[i] - self.ramp_step, self.target_motion[i])
                    changed = True
            
            if changed:
                self.send_serial_to_teensy()
            
            time.sleep(self.ramp_delay)

    def send_serial_to_teensy(self):
        if self.ser:
            # Kirim Format: M:S,Y,H,R,T,TiltArm,Gripper
            cmd = "M:" + ",".join(map(str, self.current_motion)) + "\n"
            self.ser.write(cmd.encode('utf-8'))

    # ================= BACA SENSOR & PWM DARI TEENSY =================
    def serial_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 1. Parsing Sensor (IMU & Depth)
                    if line.startswith("P:"):
                        parts = line.split()
                        if len(parts) >= 4:
                            try:
                                imu_msg = Vector3(
                                    x=float(parts[0].split(":")[1]), 
                                    y=float(parts[1].split(":")[1]), 
                                    z=float(parts[2].split(":")[1])
                                )
                                self.imu_pub.publish(imu_msg)
                                self.depth_pub.publish(Float32(float(parts[3].split(":")[1])))
                            except ValueError: pass 

                    # 2. Parsing Output PWM Asli dari Teensy -> Kirim ke GUI
                    elif line.startswith("PWM DKIRI:"):
                        # Contoh Format dari Teensy: PWM DKIRI:1500 DKANAN:1500 BKIRI:1500 BKANAN:1500 TBKIRI:1500 TBKANAN:1500
                        parts = line.replace("PWM ", "").split()
                        if len(parts) == 6:
                            try:
                                d_kiri  = int(parts[0].split(":")[1])
                                d_kanan = int(parts[1].split(":")[1])
                                b_kiri  = int(parts[2].split(":")[1])
                                b_kanan = int(parts[3].split(":")[1])
                                t_kiri  = int(parts[4].split(":")[1]) # TBKIRI
                                t_kanan = int(parts[5].split(":")[1]) # TBKANAN
                                
                                # Mengurutkan sesuai susunan di callback GUI: 
                                # [DKIRI, TKIRI, BKIRI, DKANAN, TKANAN, BKANAN]
                                pwm_msg = Int32MultiArray()
                                pwm_msg.data = [d_kiri, t_kiri, b_kiri, d_kanan, t_kanan, b_kanan]
                                self.pwm_pub.publish(pwm_msg)
                            except ValueError: pass

            except OSError:
                if self.ser: self.ser.close()
                time.sleep(2)
                try:
                    port = rospy.get_param('~teensy_port', '/dev/ttyACM0')
                    baud = rospy.get_param('~teensy_baudrate', 115200)
                    self.ser = serial.Serial(port, baud, timeout=0.1)
                except Exception: pass
            except Exception: pass
                
            time.sleep(0.01)

if __name__ == '__main__':
    try:
        bridge = ThrusterBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'bridge' in locals():
            bridge.running = False
            if bridge.ser: bridge.ser.close()