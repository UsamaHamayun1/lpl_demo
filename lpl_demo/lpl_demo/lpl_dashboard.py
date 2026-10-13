#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty, Bool
import cv2
import json
import numpy as np

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, 
                             QPushButton, QHBoxLayout, QFrame, QProgressBar, QGridLayout)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QImage, QPixmap, QKeyEvent

# --- CONFIGURATION ---
WINDOW_TITLE = "LPL MISSION CONTROL + METRICS"
REFRESH_RATE_MS = 30 

class LPLDashboard(QWidget):
    def __init__(self):
        super().__init__()
        
        # 1. ROS SETUP
        rclpy.init(args=None)
        self.node = Node('lpl_mission_control_qt')
        
        # Subscribers
        self.sub_status = self.node.create_subscription(String, '/lpl/status', self.status_callback, 10)
        self.sub_cam = self.node.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publishers
        self.pub_auth = self.node.create_publisher(Empty, '/lpl/authorize', 10)
        self.pub_input = self.node.create_publisher(Twist, '/cmd_vel_input', 10)
        self.pub_actor = self.node.create_publisher(String, '/lpl/actor_mode', 10)
        self.pub_lpl_mode = self.node.create_publisher(Bool, '/lpl/activation', 10)
        self.pub_blue = self.node.create_publisher(Twist, '/blue_cmd_vel', 10)

        # State Variables
        self.manual_mode = False
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.lpl_enabled = True # Default to ON

        # 2. UI SETUP
        self.init_ui()

        # 3. TIMERS
        self.timer_ros = QTimer(self)
        self.timer_ros.timeout.connect(self.spin_ros)
        self.timer_ros.start(REFRESH_RATE_MS)

        self.timer_drive = QTimer(self)
        self.timer_drive.timeout.connect(self.publish_drive)
        self.timer_drive.start(100) # 10Hz Control Loop

    def init_ui(self):
        self.setWindowTitle(WINDOW_TITLE)
        self.setGeometry(100, 100, 900, 900) 
        self.setStyleSheet("background-color: #121212; color: #e0e0e0;")

        layout = QVBoxLayout()
        layout.setSpacing(15)
        layout.setContentsMargins(30, 30, 30, 30)

        # --- HEADER ---
        header = QLabel("ROBOT OPTICAL FEED")
        header.setAlignment(Qt.AlignCenter)
        header.setFont(QFont("Arial", 10, QFont.Bold))
        header.setStyleSheet("color: #888;")
        layout.addWidget(header)

        # --- VIDEO FEED ---
        self.lbl_video = QLabel("NO SIGNAL\n(Check Gazebo)")
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setStyleSheet("background-color: black; border: 2px solid #333; border-radius: 5px;")
        self.lbl_video.setMinimumSize(800, 350)
        self.lbl_video.setSizePolicy(self.lbl_video.sizePolicy().Expanding, self.lbl_video.sizePolicy().Expanding)
        layout.addWidget(self.lbl_video)

        # --- STATUS & TELEMETRY PANEL ---
        status_frame = QFrame()
        status_frame.setStyleSheet("background-color: #1e1e1e; border-radius: 10px; padding: 15px;")
        grid_layout = QGridLayout(status_frame)
        grid_layout.setSpacing(10)

        self.lbl_obj_title = QLabel("TARGET: --")
        self.lbl_obj_title.setFont(QFont("Courier New", 14, QFont.Bold))
        grid_layout.addWidget(self.lbl_obj_title, 0, 0)

        self.lbl_state = QLabel("WAITING")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setFont(QFont("Arial", 22, QFont.Bold))
        self.lbl_state.setStyleSheet("color: gray;")
        grid_layout.addWidget(self.lbl_state, 0, 1)

        self.lbl_conf_val = QLabel("CONF: 1.00")
        self.lbl_conf_val.setAlignment(Qt.AlignRight)
        self.lbl_conf_val.setFont(QFont("Courier New", 14, QFont.Bold))
        grid_layout.addWidget(self.lbl_conf_val, 0, 2)

        self.bar_conf = QProgressBar()
        self.bar_conf.setRange(0, 100)
        self.bar_conf.setValue(100)
        self.bar_conf.setTextVisible(False)
        self.bar_conf.setFixedHeight(12)
        self.bar_conf.setStyleSheet("QProgressBar::chunk { background-color: #28a745; }")
        grid_layout.addWidget(self.bar_conf, 1, 0, 1, 3) 

        # Proximity
        lbl_prox = QLabel("PROXIMITY (Distance):")
        lbl_prox.setStyleSheet("color: #aaa; font-weight: bold;")
        grid_layout.addWidget(lbl_prox, 2, 0)

        self.bar_prox = QProgressBar()
        self.bar_prox.setRange(0, 100)
        self.bar_prox.setValue(100)
        self.bar_prox.setFixedHeight(8) 
        self.bar_prox.setTextVisible(False)
        self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #00ccff; }")
        grid_layout.addWidget(self.bar_prox, 2, 1, 1, 2) 

        # Stability
        lbl_stab = QLabel("STABILITY (Behavior):")
        lbl_stab.setStyleSheet("color: #aaa; font-weight: bold;")
        grid_layout.addWidget(lbl_stab, 3, 0)

        self.bar_stab = QProgressBar()
        self.bar_stab.setRange(0, 100)
        self.bar_stab.setValue(100)
        self.bar_stab.setFixedHeight(8) 
        self.bar_stab.setTextVisible(False)
        self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #ffcc00; }")
        grid_layout.addWidget(self.bar_stab, 3, 1, 1, 2) 

        # Velocity
        vel_label = QLabel("VELOCITY:")
        vel_label.setFont(QFont("Arial", 10, QFont.Bold))
        vel_label.setStyleSheet("color: #666; margin-top: 10px;")
        grid_layout.addWidget(vel_label, 4, 0)

        self.lbl_vel_lin = QLabel("LIN: 0.00 m/s")
        self.lbl_vel_lin.setFont(QFont("Courier New", 12))
        self.lbl_vel_lin.setStyleSheet("color: #00ccff; margin-top: 10px;")
        grid_layout.addWidget(self.lbl_vel_lin, 4, 1)

        self.lbl_vel_ang = QLabel("ANG: 0.00 rad/s")
        self.lbl_vel_ang.setFont(QFont("Courier New", 12))
        self.lbl_vel_ang.setStyleSheet("color: #ffcc00; margin-top: 10px;")
        grid_layout.addWidget(self.lbl_vel_ang, 4, 2, Qt.AlignRight)

        layout.addWidget(status_frame)

        # --- LPL SYSTEM TOGGLE ---
        sys_layout = QHBoxLayout()
        self.btn_lpl_toggle = QPushButton("LPL SYSTEM: ON (Filtering Active)")
        self.btn_lpl_toggle.setFixedHeight(50)
        self.btn_lpl_toggle.setFont(QFont("Arial", 12, QFont.Bold))
        self.btn_lpl_toggle.setStyleSheet("background-color: #28a745; color: white; border-radius: 5px;")
        self.btn_lpl_toggle.clicked.connect(self.toggle_lpl)
        sys_layout.addWidget(self.btn_lpl_toggle)
        layout.addLayout(sys_layout)

        # --- ROBOT & BLUE BOX CONTROLS ---
        btn_layout = QHBoxLayout()
        lbl_instruct = QLabel("ROBOT: WASD to Drive, X to Reverse, SPACE to Take Control\nBLUE HAZARD: U=Fwd, N=Back, H=Left, J=Right, K=Stop")
        lbl_instruct.setFont(QFont("Arial", 10, QFont.Bold))
        lbl_instruct.setStyleSheet("color: #888;")
        btn_layout.addWidget(lbl_instruct)
        
        btn_layout.addStretch()

        self.btn_stop = QPushButton("EMERGENCY STOP")
        self.btn_stop.setFixedSize(200, 60)
        self.btn_stop.setFont(QFont("Arial", 12, QFont.Bold))
        self.btn_stop.setStyleSheet("background-color: #d32f2f; color: white; border-radius: 5px;")
        self.btn_stop.clicked.connect(lambda: self.set_speed(0.0, 0.0))
        btn_layout.addWidget(self.btn_stop)

        self.btn_auth = QPushButton("TAKE CONTROL\n[SPACE]")
        self.btn_auth.setFixedSize(250, 60)
        self.btn_auth.setFont(QFont("Arial", 14, QFont.Bold))
        self.btn_auth.setStyleSheet("background-color: #444; color: #888; border-radius: 5px;")
        self.btn_auth.setEnabled(False)
        self.btn_auth.clicked.connect(self.send_auth)
        btn_layout.addWidget(self.btn_auth)

        layout.addLayout(btn_layout)

        # --- SEPARATOR ---
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("background-color: #333;")
        layout.addWidget(line)

        # --- ACTOR CONTROLS ---
        actor_layout = QHBoxLayout()
        lbl_actor = QLabel("SIMULATION\nCONTROL:")
        lbl_actor.setFont(QFont("Arial", 10, QFont.Bold))
        lbl_actor.setStyleSheet("color: #888;")
        actor_layout.addWidget(lbl_actor)

        self.btn_stable = QPushButton("STABLE MODE (Normal)")
        self.btn_stable.setFixedSize(250, 50)
        self.btn_stable.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_stable.setStyleSheet("background-color: #007bff; color: white; border-radius: 5px;")
        self.btn_stable.clicked.connect(lambda: self.set_actor_mode("SMOOTH"))
        actor_layout.addWidget(self.btn_stable)

        self.btn_jitter = QPushButton("JITTER MODE (High Variance)")
        self.btn_jitter.setFixedSize(250, 50)
        self.btn_jitter.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_jitter.setStyleSheet("background-color: #fd7e14; color: white; border-radius: 5px;")
        self.btn_jitter.clicked.connect(lambda: self.set_actor_mode("JITTER"))
        actor_layout.addWidget(self.btn_jitter)
        
        layout.addLayout(actor_layout)
        self.setLayout(layout)
        self.setFocusPolicy(Qt.StrongFocus)

    # --- LOGIC ---

    def toggle_lpl(self):
        self.lpl_enabled = not self.lpl_enabled
        msg = Bool()
        msg.data = self.lpl_enabled
        self.pub_lpl_mode.publish(msg)
        
        if self.lpl_enabled:
            self.btn_lpl_toggle.setText("LPL SYSTEM: ON (Filtering Active)")
            self.btn_lpl_toggle.setStyleSheet("background-color: #28a745; color: white; border-radius: 5px;")
        else:
            self.btn_lpl_toggle.setText("LPL SYSTEM: OFF (Raw Nav2 Active)")
            self.btn_lpl_toggle.setStyleSheet("background-color: #d32f2f; color: white; border-radius: 5px;")

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def set_blue_speed(self, x, y):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y) # Planar move uses Y for side-to-side
        self.pub_blue.publish(msg)

    def publish_drive(self):
        msg = Twist()
        msg.linear.x = float(self.target_lin)
        msg.angular.z = float(self.target_ang)
        self.pub_input.publish(msg)
        self.lbl_vel_lin.setText(f"LIN: {self.target_lin:.2f} m/s")
        self.lbl_vel_ang.setText(f"ANG: {self.target_ang:.2f} rad/s")

    def send_auth(self):
        if self.btn_auth.isEnabled() or self.manual_mode:
            self.pub_auth.publish(Empty())

    def set_speed(self, x, z):
        self.target_lin = x
        self.target_ang = z
    
    def set_actor_mode(self, mode):
        msg = String()
        msg.data = mode
        self.pub_actor.publish(msg)

    # --- EVENTS ---
    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        # Robot Controls
        if key == Qt.Key_W: self.set_speed(0.3, 0.0)
        elif key == Qt.Key_S: self.set_speed(0.0, 0.0) 
        elif key == Qt.Key_A: self.set_speed(0.0, 0.5) 
        elif key == Qt.Key_D: self.set_speed(0.0, -0.5)
        elif key == Qt.Key_X: self.set_speed(-0.2, 0.0)
        elif key == Qt.Key_Space: self.send_auth()

        # Blue Box Controls
        elif key == Qt.Key_U: self.set_blue_speed(1.0, 0.0)  # Forward
        elif key == Qt.Key_N: self.set_blue_speed(-1.0, 0.0) # Backward
        elif key == Qt.Key_H: self.set_blue_speed(0.0, 1.0)  # Left
        elif key == Qt.Key_J: self.set_blue_speed(0.0, -1.0) # Right
        elif key == Qt.Key_K: self.set_blue_speed(0.0, 0.0)  # Stop

    # --- CALLBACKS ---
    # def status_callback(self, msg):
    #     try:
    #         data = json.loads(msg.data)
    #         conf = float(data.get('confidence', 0.0))
    #         state = data.get('state', "UNKNOWN")
    #         obj = data.get('object', "--")
    #         self.manual_mode = data.get('override', False)
            
    #         prox = float(data.get('prox', 1.0)) 
    #         stab = float(data.get('stab', 1.0))

    #         self.lbl_obj_title.setText(f"TARGET: {obj.upper()}")
    #         self.lbl_state.setText(state)
    #         self.lbl_conf_val.setText(f"CONF: {conf:.2f}")
    #         self.bar_conf.setValue(int(conf * 100))
            
    #         self.bar_prox.setValue(int(prox * 100))
    #         self.bar_stab.setValue(int(stab * 100))

    #         # --- Proximity & Stability Mini-Bar Coloring ---
    #         if prox < 0.3: self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #ff3333; }")
    #         elif prox < 0.6: self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #ffcc00; }")
    #         else: self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #00ccff; }")
            
    #         if stab < 0.3: self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #ff3333; }")
    #         elif stab < 0.6: self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #ffcc00; }")
    #         else: self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #aa00ff; }")

    #         # --- LPL COLOR STATE MAPPING (Backgrounds/Labels Only) ---
    #         if state == "CONFIDENT" or state == "LPL DISABLED":
    #             c = "#28a745" # GREEN
    #             self.lbl_state.setStyleSheet(f"color: {c}; font-weight: bold;")
    #             self.bar_conf.setStyleSheet(f"QProgressBar::chunk {{ background-color: {c}; }}")

    #         elif state == "CAUTIOUS":
    #             c = "#ffc107" # YELLOW
    #             self.lbl_state.setStyleSheet(f"color: {c}; font-weight: bold;")
    #             self.bar_conf.setStyleSheet(f"QProgressBar::chunk {{ background-color: {c}; }}")

    #         elif state == "PAUSED" or state == "DANGER STOP":
    #             c = "#dc3545" # RED
    #             self.lbl_state.setStyleSheet(f"color: {c}; font-weight: bold;")
    #             self.bar_conf.setStyleSheet(f"QProgressBar::chunk {{ background-color: {c}; }}")

    #         elif state == "MANUAL OVERRIDE":
    #             c = "#007bff" # BLUE
    #             self.lbl_state.setStyleSheet(f"color: {c}; font-weight: bold;")
    #             self.bar_conf.setStyleSheet(f"QProgressBar::chunk {{ background-color: {c}; }}")

    #         # --- BUTTON OVERRIDE LOGIC ---
    #         # If the human has authority, the button is ALWAYS Blue and active
    #         if self.manual_mode: 
    #             self.btn_auth.setStyleSheet("background-color: #007bff; color: white; font-weight: bold; border-radius: 5px;")
    #             self.btn_auth.setText("AUTHORIZE AI RESUME\n[SPACE]")
    #             self.btn_auth.setEnabled(True)
    #         else:
    #             # If AI has authority, button is gray and inactive
    #             self.btn_auth.setStyleSheet("background-color: #444; color: #888; border-radius: 5px;")
    #             self.btn_auth.setEnabled(False)
    #             if state == "LPL DISABLED": self.btn_auth.setText("OFFLINE")
    #             elif state == "CONFIDENT": self.btn_auth.setText("ALL CLEAR")
    #             elif state == "CAUTIOUS": self.btn_auth.setText("CAUTION")
    #             elif state == "PAUSED" or state == "DANGER STOP": self.btn_auth.setText("PAUSED")
    #             elif state == "MANUAL OVERRIDE": self.btn_auth.setText("AUTHORIZING...")

    #     except Exception:
    #         pass
    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            conf = float(data.get('confidence', 0.0))
            state = data.get('state', "UNKNOWN")
            obj = data.get('object', "--")
            self.manual_mode = data.get('override', False)
            
            prox = float(data.get('prox', 1.0)) 
            stab = float(data.get('stab', 1.0))

            self.lbl_obj_title.setText(f"TARGET: {obj.upper()}")
            self.lbl_state.setText(state)
            self.lbl_conf_val.setText(f"CONF: {conf:.2f}")
            self.bar_conf.setValue(int(conf * 100))
            
            self.bar_prox.setValue(int(prox * 100))
            self.bar_stab.setValue(int(stab * 100))

            # --- Proximity & Stability Mini-Bar Coloring ---
            if prox < 0.3: self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #ff3333; }")
            elif prox < 0.6: self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #ffcc00; }")
            else: self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #00ccff; }")
            
            if stab < 0.3: self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #ff3333; }")
            elif stab < 0.6: self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #ffcc00; }")
            else: self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #aa00ff; }")

            # --- PROGRESS BAR SLIDING COLOR LOGIC ---
            # This ensures the bar visually slides through the colors as the numeric value drops!
            if self.manual_mode and conf <= 0.05:
                bar_color = "#007bff" # BLUE (Only turns blue when fully drained and human is ready)
            elif conf <= 0.20:
                bar_color = "#dc3545" # RED (Draining to empty)
            elif conf < 0.90:
                bar_color = "#ffc107" # YELLOW (Passing through middle)
            else:
                bar_color = "#28a745" # GREEN (Confident)
            
            self.bar_conf.setStyleSheet(f"QProgressBar::chunk {{ background-color: {bar_color}; }}")

            # --- TEXT & BUTTON STATE MAPPING ---
            if state == "CONFIDENT" or state == "LPL DISABLED":
                self.lbl_state.setStyleSheet("color: #28a745; font-weight: bold;")
            elif state == "CAUTIOUS":
                self.lbl_state.setStyleSheet("color: #ffc107; font-weight: bold;")
            elif state == "PAUSED" or state == "DANGER STOP":
                self.lbl_state.setStyleSheet("color: #dc3545; font-weight: bold;")
            elif state == "MANUAL OVERRIDE":
                self.lbl_state.setStyleSheet("color: #007bff; font-weight: bold;")

            # --- BUTTON OVERRIDE LOGIC ---
            if self.manual_mode: 
                self.btn_auth.setStyleSheet("background-color: #007bff; color: white; font-weight: bold; border-radius: 5px;")
                self.btn_auth.setText("AUTHORIZE RESUME\n[SPACE]")
                self.btn_auth.setEnabled(True)
            else:
                self.btn_auth.setStyleSheet("background-color: #444; color: #888; border-radius: 5px;")
                self.btn_auth.setEnabled(False)
                if state == "LPL DISABLED": self.btn_auth.setText("OFFLINE")
                elif state == "CONFIDENT": self.btn_auth.setText("ALL CLEAR")
                elif state == "CAUTIOUS": self.btn_auth.setText("CAUTION")
                elif state == "PAUSED" or state == "DANGER STOP": self.btn_auth.setText("PAUSED")
                elif state == "MANUAL OVERRIDE": self.btn_auth.setText("AUTHORIZING...")

        except Exception:
            pass
    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = np_arr.reshape(msg.height, msg.width, -1)
            if 'rgb' in msg.encoding: cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            scaled_pixmap = QPixmap.fromImage(qt_img).scaled(
                self.lbl_video.width(), self.lbl_video.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.lbl_video.setPixmap(scaled_pixmap)
        except Exception: pass

    def closeEvent(self, event):
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = LPLDashboard()
    window.show()   
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()