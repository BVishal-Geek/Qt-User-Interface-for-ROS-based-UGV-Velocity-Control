#!/usr/bin/env python3
#######################################################################################################
#                                            Imports 
######################################################################################################
from __future__ import division

from matplotlib.pyplot import get
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from python_qt_binding.binding_helper import _load_pyqt
import rospy
import cv2
import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer
import numpy as np 
import subprocess
import signal
from pathlib import Path
import os
import time


#################################################################################################################################
#                                              Class Declaration
#############################################################################################################################
class Ui_Form(QtWidgets.QWidget,object):

    slider_factor = 1000
    voltage_constant_update = 0
    Get_ip_from_lineEdit = None
    def setupUi(self, Form):
################################################################################################################################
                                        # Widgets # 
###############################################################################################################################

        Form.setObjectName("Form")
        Form.resize(1144, 712)
        Form.setSizeIncrement(QtCore.QSize(710, 510))
        Form.setBaseSize(QtCore.QSize(435, 200))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("../../../../Downloads/WhatsApp Image 2022-04-07 at 2.51.33 PM.jpeg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        Form.setWindowIcon(icon)
        self.horizontalLayout = QtWidgets.QHBoxLayout(Form)
        self.horizontalLayout.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.SlamLaunch = QtWidgets.QTabWidget(Form)
        self.SlamLaunch.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.SlamLaunch.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.SlamLaunch.setObjectName("SlamLaunch")
        self.Control = QtWidgets.QWidget()
        self.Control.setObjectName("Control")
        self.Speed_linear_x = QtWidgets.QSlider(self.Control)
        self.Speed_linear_x.setGeometry(QtCore.QRect(550, 20, 20, 511))
        self.Speed_linear_x.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.Speed_linear_x.setMinimum(-1000)
        self.Speed_linear_x.setMaximum(1000)
        self.Speed_linear_x.setSingleStep(10)
        self.Speed_linear_x.setPageStep(100)
        self.Speed_linear_x.setOrientation(QtCore.Qt.Vertical)
        self.Speed_linear_x.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.Speed_linear_x.setTickInterval(0)
        self.Speed_linear_x.setObjectName("Speed_linear_x")
        self.angular_speed = QtWidgets.QSlider(self.Control)
        self.angular_speed.setGeometry(QtCore.QRect(80, 580, 961, 20))
        self.angular_speed.setMinimum(-1000)
        self.angular_speed.setMaximum(1000)
        self.angular_speed.setSingleStep(10)
        self.angular_speed.setPageStep(100)
        self.angular_speed.setOrientation(QtCore.Qt.Horizontal)
        self.angular_speed.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.angular_speed.setTickInterval(0)
        self.angular_speed.setObjectName("angular_speed")
        self.linear_stop = QtWidgets.QPushButton(self.Control)
        self.linear_stop.setGeometry(QtCore.QRect(490, 260, 41, 31))
        self.linear_stop.setObjectName("linear_stop")
        self.angular_stop = QtWidgets.QPushButton(self.Control)
        self.angular_stop.setGeometry(QtCore.QRect(540, 610, 41, 31))
        self.angular_stop.setObjectName("angular_stop")
        self.control_combo_box = QtWidgets.QComboBox(self.Control)
        self.control_combo_box.setGeometry(QtCore.QRect(970, 20, 131, 25))
        self.control_combo_box.setObjectName("control_combo_box")
        self.control_combo_box.addItem("")
        self.control_combo_box.addItem("")
        self.linear_speed = QtWidgets.QLabel(self.Control)
        self.linear_speed.setGeometry(QtCore.QRect(600, 260, 71, 31))
        self.linear_speed.setObjectName("linear_speed")
        self.label_4 = QtWidgets.QLabel(self.Control)
        self.label_4.setGeometry(QtCore.QRect(530, 550, 81, 31))
        self.label_4.setObjectName("label_4")
        self.angular_Right = QtWidgets.QPushButton(self.Control)
        self.angular_Right.setGeometry(QtCore.QRect(1060, 570, 41, 31))
        self.angular_Right.setObjectName("angular_Right")
        self.angular_left = QtWidgets.QPushButton(self.Control)
        self.angular_left.setGeometry(QtCore.QRect(20, 570, 41, 31))
        self.angular_left.setObjectName("angular_left")
        self.linear_decrease_x = QtWidgets.QPushButton(self.Control)
        self.linear_decrease_x.setGeometry(QtCore.QRect(490, 500, 41, 31))
        self.linear_decrease_x.setObjectName("linear_decrease_x")
        self.linear_increase_x = QtWidgets.QPushButton(self.Control)
        self.linear_increase_x.setGeometry(QtCore.QRect(490, 20, 41, 31))
        self.linear_increase_x.setObjectName("linear_increase_x")
        self.Change_Linear_X_plus = QtWidgets.QDoubleSpinBox(self.Control)
        self.Change_Linear_X_plus.setGeometry(QtCore.QRect(590, 20, 81, 31))
        self.Change_Linear_X_plus.setMinimum(1.0)
        self.Change_Linear_X_plus.setObjectName("Change_Linear_X_plus")
        self.Change_Linear_X_minus = QtWidgets.QDoubleSpinBox(self.Control)
        self.Change_Linear_X_minus.setGeometry(QtCore.QRect(590, 500, 81, 31))
        self.Change_Linear_X_minus.setMinimum(-99.0)
        self.Change_Linear_X_minus.setMaximum(-1.0)
        self.Change_Linear_X_minus.setObjectName("Change_Linear_X_minus")
        self.Change_angular_Z_plus = QtWidgets.QDoubleSpinBox(self.Control)
        self.Change_angular_Z_plus.setGeometry(QtCore.QRect(960, 540, 81, 31))
        self.Change_angular_Z_plus.setMinimum(1.0)
        self.Change_angular_Z_plus.setObjectName("Change_angular_Z_plus")
        self.Change_angular_Z_minus = QtWidgets.QDoubleSpinBox(self.Control)
        self.Change_angular_Z_minus.setGeometry(QtCore.QRect(80, 540, 81, 31))
        self.Change_angular_Z_minus.setMinimum(-99.0)
        self.Change_angular_Z_minus.setMaximum(-1.0)
        self.Change_angular_Z_minus.setObjectName("Change_angular_Z_minus")
        self.camera_streaming_4 = QtWidgets.QLabel(self.Control)
        self.camera_streaming_4.setGeometry(QtCore.QRect(10, 60, 471, 431))
        self.camera_streaming_4.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.camera_streaming_4.setFrameShadow(QtWidgets.QFrame.Plain)
        self.camera_streaming_4.setLineWidth(1)
        self.camera_streaming_4.setText("")
        self.camera_streaming_4.setObjectName("camera_streaming_4")
        self.check_the_box = QtWidgets.QCheckBox(self.Control)
        self.check_the_box.setGeometry(QtCore.QRect(10, 20, 121, 23))
        self.check_the_box.setObjectName("check_the_box")
        self.Check_ping_connection = QtWidgets.QPushButton(self.Control)
        self.Check_ping_connection.setGeometry(QtCore.QRect(970, 120, 131, 25))
        self.Check_ping_connection.setObjectName("Check_ping_connection")
        self.Connection_Status = QtWidgets.QLabel(self.Control)
        self.Connection_Status.setGeometry(QtCore.QRect(770, 20, 41, 21))
        self.Connection_Status.setAutoFillBackground(False)
        self.Connection_Status.setFrameShape(QtWidgets.QFrame.Box)
        self.Connection_Status.setFrameShadow(QtWidgets.QFrame.Plain)
        self.Connection_Status.setText("")
        self.Connection_Status.setObjectName("Connection_Status")
        self.Mac_id = QtWidgets.QLabel(self.Control)
        self.Mac_id.setGeometry(QtCore.QRect(820, 20, 131, 21))
        self.Mac_id.setFrameShape(QtWidgets.QFrame.Box)
        self.Mac_id.setText("")
        self.Mac_id.setObjectName("Mac_id")
        self.lineEdit = QtWidgets.QLineEdit(self.Control)
        self.lineEdit.setGeometry(QtCore.QRect(970, 90, 131, 25))
        self.lineEdit.setObjectName("lineEdit")
        self.SlamLaunch.addTab(self.Control, "")
        self.Odom = QtWidgets.QWidget()
        self.Odom.setObjectName("Odom")
        self.motor1_name = QtWidgets.QLabel(self.Odom)
        self.motor1_name.setGeometry(QtCore.QRect(30, 57, 67, 20))
        self.motor1_name.setObjectName("motor1_name")
        self.motor2_name = QtWidgets.QLabel(self.Odom)
        self.motor2_name.setGeometry(QtCore.QRect(480, 60, 67, 17))
        self.motor2_name.setObjectName("motor2_name")
        self.motor4_name = QtWidgets.QLabel(self.Odom)
        self.motor4_name.setGeometry(QtCore.QRect(480, 310, 67, 17))
        self.motor4_name.setObjectName("motor4_name")
        self.motor3_name = QtWidgets.QLabel(self.Odom)
        self.motor3_name.setGeometry(QtCore.QRect(30, 310, 67, 17))
        self.motor3_name.setObjectName("motor3_name")
        self.Battery_Status = QtWidgets.QProgressBar(self.Odom)
        self.Battery_Status.setGeometry(QtCore.QRect(940, 10, 161, 31))
        self.Battery_Status.setFocusPolicy(QtCore.Qt.WheelFocus)
        self.Battery_Status.setProperty("value", 0)
        self.Battery_Status.setObjectName("Battery_Status")
        self.voltage_consumption = QtWidgets.QLCDNumber(self.Odom)
        self.voltage_consumption.setGeometry(QtCore.QRect(940, 100, 161, 51))
        self.voltage_consumption.setObjectName("voltage_consumption")
        self.Current_consumption = QtWidgets.QLCDNumber(self.Odom)
        self.Current_consumption.setGeometry(QtCore.QRect(940, 210, 161, 51))
        self.Current_consumption.setObjectName("Current_consumption")
        self.label = QtWidgets.QLabel(self.Odom)
        self.label.setGeometry(QtCore.QRect(940, 70, 67, 17))
        self.label.setObjectName("label")
        self.label_13 = QtWidgets.QLabel(self.Odom)
        self.label_13.setGeometry(QtCore.QRect(940, 180, 67, 17))
        self.label_13.setObjectName("label_13")
        self.motor_speed_1 = QtWidgets.QLCDNumber(self.Odom)
        self.motor_speed_1.setGeometry(QtCore.QRect(30, 90, 141, 101))
        self.motor_speed_1.setObjectName("motor_speed_1")
        self.ok_motor_3 = QtWidgets.QLabel(self.Odom)
        self.ok_motor_3.setGeometry(QtCore.QRect(190, 370, 109, 49))
        self.ok_motor_3.setFrameShape(QtWidgets.QFrame.Box)
        self.ok_motor_3.setObjectName("ok_motor_3")
        self.ok_motor_1 = QtWidgets.QLabel(self.Odom)
        self.ok_motor_1.setGeometry(QtCore.QRect(190, 120, 109, 49))
        self.ok_motor_1.setFrameShape(QtWidgets.QFrame.Box)
        self.ok_motor_1.setObjectName("ok_motor_1")
        self.ok_motor_2 = QtWidgets.QLabel(self.Odom)
        self.ok_motor_2.setGeometry(QtCore.QRect(640, 120, 109, 49))
        self.ok_motor_2.setFrameShape(QtWidgets.QFrame.Box)
        self.ok_motor_2.setObjectName("ok_motor_2")
        self.ok_motor_4 = QtWidgets.QLabel(self.Odom)
        self.ok_motor_4.setGeometry(QtCore.QRect(640, 370, 109, 49))
        self.ok_motor_4.setFrameShape(QtWidgets.QFrame.Box)
        self.ok_motor_4.setObjectName("ok_motor_4")
        self.motor_speed_4 = QtWidgets.QLCDNumber(self.Odom)
        self.motor_speed_4.setGeometry(QtCore.QRect(480, 340, 141, 101))
        self.motor_speed_4.setObjectName("motor_speed_4")
        self.motor_speed_3 = QtWidgets.QLCDNumber(self.Odom)
        self.motor_speed_3.setGeometry(QtCore.QRect(30, 340, 141, 101))
        self.motor_speed_3.setObjectName("motor_speed_3")
        self.motor_speed_2 = QtWidgets.QLCDNumber(self.Odom)
        self.motor_speed_2.setGeometry(QtCore.QRect(480, 90, 141, 101))
        self.motor_speed_2.setMouseTracking(True)
        self.motor_speed_2.setToolTipDuration(-1)
        self.motor_speed_2.setMode(QtWidgets.QLCDNumber.Dec)
        self.motor_speed_2.setProperty("intValue", 0)
        self.motor_speed_2.setObjectName("motor_speed_2")
        self.Start_map = QtWidgets.QPushButton(self.Odom)
        self.Start_map.setGeometry(QtCore.QRect(300, 490, 111, 71))
        self.Start_map.setObjectName("Start_map")
        self.Save_Map = QtWidgets.QPushButton(self.Odom)
        self.Save_Map.setGeometry(QtCore.QRect(540, 490, 111, 71))
        self.Save_Map.setObjectName("Save_Map")
        self.Map_name_const = QtWidgets.QLabel(self.Odom)
        self.Map_name_const.setGeometry(QtCore.QRect(700, 510, 81, 31))
        self.Map_name_const.setObjectName("Map_name_const")
        self.Map_path_const = QtWidgets.QLabel(self.Odom)
        self.Map_path_const.setGeometry(QtCore.QRect(700, 550, 81, 31))
        self.Map_path_const.setObjectName("Map_path_const")
        self.Note_points = QtWidgets.QLabel(self.Odom)
        self.Note_points.setGeometry(QtCore.QRect(700, 590, 401, 20))
        self.Note_points.setObjectName("Note_points")
        self.Exit_Mapping = QtWidgets.QPushButton(self.Odom)
        self.Exit_Mapping.setGeometry(QtCore.QRect(420, 580, 111, 71))
        self.Exit_Mapping.setObjectName("Exit_Mapping")
        self.Map_name_input = QtWidgets.QLineEdit(self.Odom)
        self.Map_name_input.setGeometry(QtCore.QRect(790, 510, 251, 31))
        self.Map_name_input.setObjectName("Map_name_input")
        self.map_path_input = QtWidgets.QLineEdit(self.Odom)
        self.map_path_input.setGeometry(QtCore.QRect(790, 550, 251, 31))
        self.map_path_input.setObjectName("map_path_input")
        self.SlamLaunch.addTab(self.Odom, "")
        self.horizontalLayout.addWidget(self.SlamLaunch)

        self.retranslateUi(Form)
        self.SlamLaunch.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Form)

###########################################################################################################
                        # Functions to be executed when UI calls 
###########################################################################################################
        self.linear_stop.clicked.connect(self.speed_init)
        self.angular_stop.clicked.connect(self.angular_init)
        self.Speed_linear_x.valueChanged.connect(self._on_x_linear_slider_changed)
        self.angular_speed.valueChanged.connect(self._on_z_angular_slider_changed)
        
        self.linear_increase_x.clicked.connect(self._on_plus_linear_button_pressed)
        self.Change_Linear_X_plus.valueChanged.connect(self._on_max_linear_x_changed)
        self.Change_Linear_X_minus.valueChanged.connect(self._on_min_linear_x_changed)
        self.Change_angular_Z_plus.valueChanged.connect(self._on_max_angular_changed)
        self.Change_angular_Z_minus.valueChanged.connect(self._on_min_angular_changed)
        self.linear_decrease_x.clicked.connect(self._on_minus_linear_button_pressed)
        self.angular_left.clicked.connect(self._on_left_angular_button_pressed)
        self.angular_Right.clicked.connect(self._on_right_angular_button_pressed)
        

        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(self._on_topic_changed)
        self._update_parameter_timer.start(100) 
        
        self.control_combo_box.currentTextChanged.connect(self._on_topic_changed)
        self.control_combo_box.currentTextChanged.connect(self._on_topic_changed_1)
        self.Check_ping_connection.clicked.connect(self.Connection_Status_Update)
        self.Start_map.clicked.connect(self.start_Mapping)
        self.Exit_Mapping.clicked.connect(self.kill_mapping)
        self.Save_Map.clicked.connect(self.save_map)
        self.Worker = worker1()
        self.Worker.start()
        self.Worker.FloatUpdate.connect(self.FloatConstUpdate)
        

        self.Get_Ip_From_Class = Throw_Ip()
        self.Get_Ip_From_Class.start()
        self.Get_Ip_From_Class.color_update.connect(self.Connection_Status_Update)
       

    #################################################################################################################################
                                        # ROS Node Initialization ##
#################################################################################################################################
        rospy.init_node('APOSTLE_UI',anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=50)
        self.sub = rospy.Subscriber('/raspicam_node/image/compressed',CompressedImage, self.Image_callback)
        self.motor_1_Encoder = rospy.Subscriber('enc1',String,self.motor_1_callback)
        self.motor_2_Encoder = rospy.Subscriber('enc2',String,self.motor_2_callback)
        self.motor_3_Encoder = rospy.Subscriber('enc3',String,self.motor_3_callback)
        self.motor_4_Encoder = rospy.Subscriber('enc4',String,self.motor_4_callback)
        self.voltage_status = rospy.Subscriber('/voltage',Float32,self.voltage_callback)
        self.current_status = rospy.Subscriber('/current',Float32,self.Current_callback)
        self.Mac_Addr = rospy.Subscriber('/MAC_ADDR',String,self.get_mac_addr)
        rate = rospy.Rate(20)
    
    
#################################################################################################################################
                                    # Functions
#################################################################################################################################

    def Image_callback(self,msg):
       
        if (self.check_the_box.isChecked()): 
            self.video = msg.data
            np_arr = np.frombuffer(self.video,np.uint8)
            image = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
            ims = cv2.resize(image,(620,620))
            rgb_image = cv2.cvtColor(ims, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            self.convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            self.camera_streaming_4.setPixmap(QPixmap.fromImage(self.convert_to_Qt_format))
        
        else: 
            self.camera_streaming_4.setText(" ")
        
    def get_mac_addr(self,Mac_Id):
        self.Get_Mac_Id = Mac_Id.data
        self.Mac_id.setText(self.Get_Mac_Id)
        # print(self.Get_Mac_Id)
        self.Mac_Addr.unregister()


    
    def FloatConstUpdate(self,battery_percentage):
        self.Battery_Status.setValue(int(battery_percentage))


    def Connection_Status_Update(self,Status_update):
        if Status_update == 0:
            print("Connected")
        else:
            print("not connected")
        
    # def Check_ip(self):
    #     get_ip_id = int(self.lineEdit.text())
    #     print(type(get_ip_id))
        


    

        
        
    def _on_topic_changed(self):
        
        drop_box = self.control_combo_box.currentText()
        
        if (drop_box == "/cmd_vel"):
            
            vel_msg = Twist()
            vel_msg.linear.x = self.Speed_linear_x.value()/self.slider_factor
            vel_msg.angular.z = -self.angular_speed.value()/self.slider_factor
            
            if not rospy.is_shutdown():
                self.pub.publish(vel_msg)
                                
            
    def _on_topic_changed_1(self):
        drop_box = self.control_combo_box.currentText()
        if ((drop_box == "/cmd_vel") and (self.Speed_linear_x.value() > 0 or self.Speed_linear_x.value() < 0) and (self.angular_speed.value() > 0 or self.angular_speed.value() < 0)):
            
            self.Speed_linear_x.setValue(0)
            self.angular_speed.setValue(0)
                       

        if (((drop_box == "/cmd_vel") and (self.Speed_linear_x.value() > 0 or self.Speed_linear_x.value() < 0) and (self.angular_speed.value() == 0))):
            self.Speed_linear_x.setValue(0)
            self.angular_speed.setValue(0)

        if (((drop_box == "/cmd_vel") and (self.angular_speed.value() > 0 or self.angular_speed.value() < 0) and (self.Speed_linear_x.value() == 0))): 
            self.Speed_linear_x.setValue(0)
            self.angular_speed.setValue(0)


    def speed_init(self):
        self.Speed_linear_x.setSliderPosition(0)

    def angular_init(self):
        self.angular_speed.setSliderPosition(0)
    
    def _on_x_linear_slider_changed(self):
        self.linear_speed.setText('%0.2f m/s' % (self.Speed_linear_x.value() / self.slider_factor))


    def _on_z_angular_slider_changed(self):
        
        self.label_4.setText('%0.2f rad/s' % (self.angular_speed.value() / self.slider_factor))
    
    

    def _on_plus_linear_button_pressed(self):
        self.Speed_linear_x.setValue(self.Speed_linear_x.value() + self.Speed_linear_x.pageStep())


    def _on_minus_linear_button_pressed(self):
        self.Speed_linear_x.setValue(self.Speed_linear_x.value() - self.Speed_linear_x.pageStep())

    
    def _on_left_angular_button_pressed(self):
        self.angular_speed.setValue(self.angular_speed.value() - self.angular_speed.pageStep())

    def _on_right_angular_button_pressed(self):
        self.angular_speed.setValue(self.angular_speed.value() + self.angular_speed.pageStep())

    def _on_max_linear_x_changed(self):
        value = self.Change_Linear_X_plus.value()
        self.Speed_linear_x.setMaximum(value*self.slider_factor)

    def _on_min_linear_x_changed(self):
        value = self.Change_Linear_X_minus.value()
        self.Speed_linear_x.setMinimum(value*self.slider_factor)


    def _on_max_angular_changed(self):
        value =  self.Change_angular_Z_plus.value()
        self.angular_speed.setMaximum(value * self.slider_factor)
    
    def _on_min_angular_changed(self):
        value =  self.Change_angular_Z_minus.value()
        self.angular_speed.setMinimum(value * self.slider_factor)

    def Get_Ip(self):
        Ui_Form.Get_ip_from_lineEdit = int(self.lineEdit.text())
        
        
       
        
  
    
##########################################################################################################
                                        # MOTOR ENCODER VALUES CALLBACK
###########################################################################################################
    def motor_1_callback(self,data1):
        self.motor_speed_1.display(data1.data)

        if self.motor_speed_1.intValue() > 0 :
            self.ok_motor_1.setText("FORWARD")
        elif self.motor_speed_1.intValue() == 0:
            self.ok_motor_1.setText("NEUTRAL")
        else:
            self.ok_motor_1.setText("BACKWARD")
        
        

    def motor_2_callback(self,data2):
        self.motor_speed_2.display(data2.data)
        if self.motor_speed_2.intValue() > 0 :
            self.ok_motor_2.setText("FORWARD")
        elif self.motor_speed_2.intValue() == 0:
            self.ok_motor_2.setText("NEUTRAL")
        else:
            self.ok_motor_2.setText("BACKWARD")
        

    def motor_3_callback(self,data3):
        self.motor_speed_3.display(data3.data)

        if self.motor_speed_3.intValue() > 0 :
            self.ok_motor_3.setText("FORWARD")
        elif self.motor_speed_3.intValue() == 0:
            self.ok_motor_3.setText("NEUTRAL")
        else:
            self.ok_motor_3.setText("BACKWARD")

    def motor_4_callback(self,data4):
        self.motor_speed_4.display(data4.data)

        if self.motor_speed_4.intValue() > 0 :
            self.ok_motor_4.setText("FORWARD")
        elif self.motor_speed_4.intValue() == 0:
            self.ok_motor_4.setText("NEUTRAL")
        else:
            self.ok_motor_4.setText("BACKWARD")

    def voltage_callback(self,voltage):
        self.voltage_data = round(voltage.data,1)
        self.voltage_consumption.display(self.voltage_data)
        Ui_Form.voltage_constant_update = self.voltage_data
        
        
    def Current_callback(self,current):
        self.Current_consumption.display(current.data)
    
    def start_Mapping(self):
        self.process_navigation = subprocess.Popen(["roslaunch","--wait", "apostile_robot", "gmapping.launch"])        

    def kill_mapping(self):
        self.process_navigation.send_signal(signal.SIGINT)

    def save_map(self):
        
        self.Current_saving_directory = self.map_path_input.text()
        self.map_name = self.Map_name_input.text()
        os.system("rosrun map_server map_saver -f" + " " + os.path.join(self.Current_saving_directory,self.map_name))
        self.map_path_input.clear()
        self.Map_name_input.clear()

    
    
    
    
    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "APOSTLE"))
        self.linear_stop.setText(_translate("Form", "0"))
        self.angular_stop.setText(_translate("Form", "0"))
        self.control_combo_box.setItemText(0, _translate("Form", "None"))
        self.control_combo_box.setItemText(1, _translate("Form", "/cmd_vel"))
        self.linear_speed.setText(_translate("Form", "0.0 m/s"))
        self.label_4.setText(_translate("Form", "0.0 rad/s"))
        self.angular_Right.setText(_translate("Form", ">"))
        self.angular_left.setText(_translate("Form", "<"))
        self.linear_decrease_x.setText(_translate("Form", "-"))
        self.linear_increase_x.setText(_translate("Form", "+"))
        self.check_the_box.setText(_translate("Form", "Camera Enable"))
        self.Check_ping_connection.setText(_translate("Form", "Check Connection"))
        self.SlamLaunch.setTabText(self.SlamLaunch.indexOf(self.Control), _translate("Form", "Control - Camera"))
        self.motor1_name.setText(_translate("Form", "MOTOR 1"))
        self.motor2_name.setText(_translate("Form", "MOTOR 2"))
        self.motor4_name.setText(_translate("Form", "MOTOR 4"))
        self.motor3_name.setText(_translate("Form", "MOTOR 3"))
        self.label.setText(_translate("Form", "VOLTAGE"))
        self.label_13.setText(_translate("Form", "CURRENT"))
        self.ok_motor_3.setText(_translate("Form", "           OK"))
        self.ok_motor_1.setText(_translate("Form", "           OK"))
        self.ok_motor_2.setText(_translate("Form", "           OK"))
        self.ok_motor_4.setText(_translate("Form", "           OK"))
        self.Start_map.setText(_translate("Form", "Start Mapping"))
        self.Save_Map.setText(_translate("Form", "Save"))
        self.Map_name_const.setText(_translate("Form", "Map Name:"))
        self.Map_path_const.setText(_translate("Form", "Map Path:"))
        self.Note_points.setText(_translate("Form", "NOTE:   Enter the full path, Where map has to be saved"))
        self.Exit_Mapping.setText(_translate("Form", "Exit Mapping"))
        self.SlamLaunch.setTabText(self.SlamLaunch.indexOf(self.Odom), _translate("Form", "Odometry - Battery - SLAM "))

###########################################################################################################
                    #Thread concept introduced to continously update battery percentage value
###########################################################################################################
        
class worker1(QThread):
    
    FloatUpdate = pyqtSignal(float)
    def run(self):
        class_variables_callback = Ui_Form()
        
       
        min_voltage = 8.4
        max_voltage = 14.6
      
        while True:
            self.battery_percentage = float((class_variables_callback.voltage_constant_update - min_voltage)/(max_voltage - min_voltage))*100
            self.FloatUpdate.emit(self.battery_percentage)

class Throw_Ip(QThread):
    color_update = pyqtSignal(int)
    def run(self):
        class_variables_callbackk = Ui_Form()
        
        my_over_all_ip = int(class_variables_callbackk.Get_ip_from_lineEdit) 
        proc = subprocess.Popen(['ping ','-c', '1',my_over_all_ip],
        stdout=subprocess.PIPE)
        stdout, stderr = proc.communicate()
        Status_update = proc.returncode
        
        self.color_update.emit(Status_update)

            
                
                
           
###########################################################################################################
                                        #Main Function Calls
###########################################################################################################

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
