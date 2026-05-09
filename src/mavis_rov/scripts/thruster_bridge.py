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

        # ================= ROS PUBLISHER & SUBSCRIBER =================
        self.imu_pub = rospy.Publisher('/rov/imu_euler', Vector3, queue_size=10)
        self.depth_pub = rospy.Publisher('/rov/depth', Float32, queue_size=10)
        self.pwm_pub = rospy.Publisher('/rov/thruster_pwm', Int32MultiArray, queue_size=10)

        # ================= STATE KENDALI =================
        self.mode = 1  # 1: Manual, 2: Disable, 3: Auto AI
        
        # Nilai Sumbu [S, Y, H, R, T]
        self.num_motions = 5
        self.target_motion = [1500, 1500, 1500, 1500, 1500]
        self.current_motion = [1500, 1500, 1500, 1500, 1500]
        
        self.ramp_step = 40           
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
        
        rospy.loginfo("[BRIDGE] Sistem siap! Menerima input dari Keyboard (Jetson) dan UDP Gamepad (Laptop).")

    # ================= LOGIKA MODE =================
    def set_mode(self, new_mode):
        if self.mode == new_mode: return
        self.mode = new_mode
        
        if self.mode == 1: rospy.loginfo("[BRIDGE] MODE 1: MANUAL KONTROL")
        elif self.mode == 2:
            rospy.logwarn("[BRIDGE] MODE 2: DISABLED (SAFETY CUT-OFF)")
            self.target_motion = [1500] * self.num_motions
            self.current_motion = [1500] * self.num_motions
            self.send_serial_to_teensy()
        elif self.mode == 3: rospy.loginfo("[BRIDGE] MODE 3: AUTO AI")

    def auto_ai_callback(self, msg):
        if self.mode == 3 and len(msg.data) == self.num_motions:
            self.target_motion = list(msg.data)

    # ================= UDP SERVER LOOP (MENERIMA DARI LAPTOP) =================
    def udp_server_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.udp_port))
        sock.settimeout(0.5) # Timeout 0.5 detik untuk FAILSAFE
        
        rospy.loginfo(f"[UDP] Menunggu data Gamepad di port {self.udp_port}...")
        
        while self.running and not rospy.is_shutdown():
            try:
                data, addr = sock.recvfrom(1024)
                self.udp_last_recv_time = time.time()
                
                if self.mode == 1 and not self.pressed_keys: 
                    # Jika ada tombol keyboard lokal ditekan, keyboard override gamepad sementara
                    vals = data.decode('utf-8').split(',')
                    if len(vals) == 5:
                        self.target_motion = [int(v) for v in vals]
                        
            except socket.timeout:
                # FAILSAFE: Jika laptop disconnect / kabel LAN copot, hentikan pergerakan!
                if self.mode == 1 and not self.pressed_keys and (time.time() - self.udp_last_recv_time) > 0.5:
                    self.target_motion = [1500, 1500, 1500, 1500, 1500]

    # ================= KEYBOARD FALLBACK =================
    def keyboard_listener(self):
        def on_press(key):
            try:
                if hasattr(key, 'char') and key.char in ['1', '2', '3']:
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
        
        self.target_motion = [s, y, h, r, t]

    # ================= RAMPING & TRANSMISI KE TEENSY =================
    def ramping_loop(self):
        while self.running and not rospy.is_shutdown():
            if self.mode == 2:
                time.sleep(self.ramp_delay)
                continue
            
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
            # Ekstrak nilai dari array current_motion
            s, y, h, r, t = self.current_motion
            
            # Susun string sesuai format parser C++ (S:val,Y:val,H:val,R:val,T:val\n)
            cmd = f"S:{s},Y:{y},H:{h},R:{r},T:{t}\n"
            
            self.ser.write(cmd.encode('utf-8'))

    # ================= BACA SENSOR DARI TEENSY =================
    def serial_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                # Ganti 'if' menjadi 'while' di dalam sini untuk menguras isi buffer
                while self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not line: continue
                    
                    # 1. PARSING DATA SENSOR
                    if line.startswith("P:"):
                        parts = line.split()
                        if len(parts) >= 4:
                            try:
                                p_val = float(parts[0].split(":")[1])
                                r_val = float(parts[1].split(":")[1])
                                y_val = float(parts[2].split(":")[1])
                                d_val = float(parts[3].split(":")[1])
                                
                                self.imu_pub.publish(Vector3(x=p_val, y=r_val, z=y_val))
                                self.depth_pub.publish(Float32(data=d_val))
                            except ValueError: pass
                                
                    # 2. PARSING DATA PWM
                    elif line.startswith("PWM "):
                        parts = line.replace("PWM ", "").split()
                        if len(parts) == 6:
                            try:
                                dkiri = int(parts[0].split(":")[1])
                                dkanan = int(parts[1].split(":")[1])
                                bkiri = int(parts[2].split(":")[1])
                                bkanan = int(parts[3].split(":")[1])
                                tbkiri = int(parts[4].split(":")[1])
                                tbkanan = int(parts[5].split(":")[1])
                                
                                pwm_msg = Int32MultiArray()
                                pwm_msg.data = [dkiri, tbkiri, bkiri, dkanan, tbkanan, bkanan]
                                self.pwm_pub.publish(pwm_msg)
                            except ValueError: pass

            except OSError:
                rospy.logwarn("[BRIDGE] Serial terputus. Mencoba reconnect...")
                if self.ser: self.ser.close()
                time.sleep(2)
                try:
                    port = rospy.get_param('~teensy_port', '/dev/ttyACM0')
                    baud = rospy.get_param('~teensy_baudrate', 115200)
                    self.ser = serial.Serial(port, baud, timeout=0.1)
                    rospy.loginfo("[BRIDGE] Reconnect berhasil!")
                except Exception: pass
            except Exception: pass
            
            # Tidur sebentar HANYA jika buffer sudah kosong (mencegah CPU Jetson 100%)
            time.sleep(0.005)   

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