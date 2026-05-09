#!/usr/bin/env python3
import rospy
import serial
import threading
import time
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Int32MultiArray, Float32
from pynput import keyboard

class ThrusterBridge:
    def __init__(self):
        rospy.init_node('thruster_bridge_node', anonymous=True)
        
        # ================= KONEKSI SERIAL =================
        # Mengambil parameter dari file launch (opsional, fallback ke ttyACM0)
        port = rospy.get_param('~teensy_port', '/dev/ttyACM0')
        baud = rospy.get_param('~teensy_baudrate', 115200)
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            rospy.loginfo(f"[BRIDGE] Terhubung ke Teensy di {port}")
        except Exception as e:
            rospy.logerr(f"[BRIDGE] Gagal membuka serial {port}: {e}")
            self.ser = None

        # ================= ROS PUBLISHER =================
        # Untuk mengirim data IMU ke GUI
        self.imu_pub = rospy.Publisher('/rov/imu_euler', Vector3, queue_size=10)
        self.depth_pub = rospy.Publisher('/rov/depth', Float32, queue_size=10)
        
        # ================= ROS SUBSCRIBER =================
        # Untuk menerima perintah dari AI saat di Mode 3
        rospy.Subscriber('/rov/auto_cmd', Int32MultiArray, self.auto_ai_callback)

        # ================= STATE & RAMPING =================
        self.mode = 1  # 1: Manual, 2: Disable/Safety, 3: Auto AI
        
        # Misal ROV memiliki 6 Thruster (Bisa disesuaikan jumlahnya)
        self.num_thrusters = 6
        self.target_pwm = [1500] * self.num_thrusters
        self.current_pwm = [1500] * self.num_thrusters
        
        self.ramp_step = 2               # Naik/turun 2 point per step
        self.ramp_delay = 1.0 / 90.0     # Rate 90 Hz (~0.011 detik)
        self.running = True
        
        # Variabel kontrol keyboard
        self.pressed_keys = set()

        # ================= THREADING =================
        self.serial_thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.ramp_thread = threading.Thread(target=self.ramping_loop, daemon=True)
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        
        self.serial_thread.start()
        self.ramp_thread.start()
        self.keyboard_thread.start()
        
        rospy.loginfo("[BRIDGE] Sistem siap! Mode awal: MANUAL")

    # ================= LOGIKA MODE & KENDALI =================
    def set_mode(self, new_mode):
        if self.mode == new_mode: return
        self.mode = new_mode
        
        if self.mode == 1:
            rospy.loginfo("[BRIDGE] MODE 1: MANUAL KONTROL")
        elif self.mode == 2:
            rospy.logwarn("[BRIDGE] MODE 2: DISABLED (SAFETY CUT-OFF)")
            # Instan matikan semua motor tanpa ramping
            self.target_pwm = [1500] * self.num_thrusters
            self.current_pwm = [1500] * self.num_thrusters
            self.send_pwm_to_teensy()
        elif self.mode == 3:
            rospy.loginfo("[BRIDGE] MODE 3: AUTO AI MENGAMBIL ALIH")

    def auto_ai_callback(self, msg):
        """Menerima perintah array PWM dari program AI"""
        if self.mode == 3 and len(msg.data) == self.num_thrusters:
            self.target_pwm = list(msg.data)

    # ================= LOGIKA KEYBOARD (PYNPUT) =================
    def keyboard_listener(self):
        def on_press(key):
            try:
                # Tombol ganti mode (Angka 1, 2, 3)
                if hasattr(key, 'char') and key.char in ['1', '2', '3']:
                    self.set_mode(int(key.char))
                    return

                if self.mode != 1: return # Abaikan input jika tidak di Mode 1

                if hasattr(key, 'char'):
                    k = key.char.lower()
                    self.pressed_keys.add(k)
                    self.calculate_manual_pwm()
                    
            except AttributeError:
                pass

        def on_release(key):
            if self.mode != 1: return
            try:
                if hasattr(key, 'char'):
                    k = key.char.lower()
                    if k in self.pressed_keys:
                        self.pressed_keys.remove(k)
                    self.calculate_manual_pwm()
            except AttributeError:
                pass

        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    def calculate_manual_pwm(self):
        """Menghitung target PWM berdasarkan tombol yang ditekan"""
        # Reset ke titik tengah (1500)
        pwm = [1500] * self.num_thrusters
        
        # Contoh pemetaan sederhana: W=Maju, S=Mundur, A=Kiri, D=Kanan
        if 'w' in self.pressed_keys:
            pwm = [1700, 1700, 1500, 1500, 1500, 1500] 
        elif 's' in self.pressed_keys:
            pwm = [1300, 1300, 1500, 1500, 1500, 1500]
        # (Tambahkan logika pergerakan 6-DoF Anda di sini)
        
        self.target_pwm = pwm

    # ================= LOGIKA RAMPING =================
    def ramping_loop(self):
        while self.running and not rospy.is_shutdown():
            if self.mode == 2:
                time.sleep(self.ramp_delay)
                continue
            
            changed = False
            for i in range(self.num_thrusters):
                if self.current_pwm[i] < self.target_pwm[i]:
                    self.current_pwm[i] = min(self.current_pwm[i] + self.ramp_step, self.target_pwm[i])
                    changed = True
                elif self.current_pwm[i] > self.target_pwm[i]:
                    self.current_pwm[i] = max(self.current_pwm[i] - self.ramp_step, self.target_pwm[i])
                    changed = True
            
            if changed:
                self.send_pwm_to_teensy()
            
            time.sleep(self.ramp_delay)

    def send_pwm_to_teensy(self):
        """Mengirim format data motor ke Teensy"""
        if self.ser:
            # Contoh format kirim: "M:1500,1500,1500,1500,1500,1500\n"
            cmd = "M:" + ",".join(map(str, self.current_pwm)) + "\n"
            self.ser.write(cmd.encode('utf-8'))
# ================= LOGIKA BACA SENSOR (SERIAL) =================
    def serial_loop(self):
        while self.running and not rospy.is_shutdown():
            try:
                # Mengecek apakah ada data masuk
                if self.ser and self.ser.in_waiting > 0:
                    # errors='ignore' berguna agar karakter aneh/noise tidak membuat error
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Membaca data: "P:10.5 R:-5.2 Y:120.0"
                    if line.startswith("P:"):
                        parts = line.split()
                        if len(parts) >= 3:
                            try:
                                p_val = float(parts[0].split(":")[1])
                                r_val = float(parts[1].split(":")[1])
                                y_val = float(parts[2].split(":")[1])
                                d_val = float(parts[3].split(":")[1]) 
                                
                                # Memasukkan data ke pesan ROS dan menyiarkan
                                imu_msg = Vector3()
                                imu_msg.x = p_val
                                imu_msg.y = r_val
                                imu_msg.z = y_val
                                self.imu_pub.publish(imu_msg)

                                self.depth_pub.publish(Float32(d_val))
                            except ValueError:
                                pass # Abaikan jika float gagal diparse akibat string terpotong

            except OSError as e:
                # Menangkap error [Errno 5] Input/output error
                rospy.logwarn("[BRIDGE] Serial terputus (kabel goyang/Teensy reset). Mencoba reconnect...")
                
                # Tutup port lama yang rusak
                if self.ser:
                    self.ser.close()
                time.sleep(2) # Beri jeda 2 detik agar sistem Jetson bernapas
                
                # Coba koneksi ulang
                try:
                    port = rospy.get_param('~teensy_port', '/dev/ttyACM0')
                    baud = rospy.get_param('~teensy_baudrate', 115200)
                    self.ser = serial.Serial(port, baud, timeout=0.1)
                    rospy.loginfo(f"[BRIDGE] Berhasil Reconnect ke Teensy di {port}!")
                except Exception:
                    pass # Jika gagal, loop akan berulang dan mencoba reconnect lagi nanti

            except Exception as e:
                pass # Tangkap error lain yang tidak terduga
                
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
            if bridge.ser:
                bridge.ser.close()

