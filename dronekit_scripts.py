import dronekit
import time
from PyQt5.QtCore import pyqtSignal, QThread
import math
import cv2
import numpy as np
from ultralytics import YOLO
from PyQt5.QtGui import QImage,QPixmap
import torch

vehicle=None
message=None
x_deger=0
y_deger=0
class VehicleData(QThread):
	def __init__(self):
		QThread.__init__(self)
		self.uav=None
		

	def connectVehicle(self,connection_string):
		global vehicle
		vehicle=dronekit.connect(connection_string,wait_ready=True)
		self.uav=vehicle

	def disconnectVehicle(self):
		vehicle.close()
		self.uav.close()

	def mod_degistir(self,mod):
		modes={ 'MANUAL': 0,
			'CIRCLE': 1,
			'STABILIZE':2,
			'TRAINING':3,
			'ACRO':4,
			'FBWA':5,
			'FBWB':6,
			'CRUISE':7,
			'AUTOTUNE':8,
			'AUTO':10,
			'RTL':11,
			'LOITER':12,
			'TAKEOFF':13,
			'GUIDED':15}
		uav_mode=modes.get(mod)
		_msg=self.uav.message_factory.command_long_encode(
			0,0,
			dronekit.mavutil.mavlink.MAV_CMD_DO_SET_MODE,
			0,1,
			uav_mode,
			0,0,0,0,0
		)
		self.uav.send_mavlink(_msg)
	
	

	def arm_et(self):
		global vehicle
		_msg=vehicle.message_factory.command_long_encode(
			0,0,
			dronekit.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
			0,
			1, #0 disarm 1 arm
			0, #21196 force arm,disarm
			0,0,0,0,0
		)
		vehicle.send_mavlink(_msg)

	def disarm_et(self):
		_msg=vehicle.message_factory.command_long_encode(
			0,0,
			dronekit.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
			0,
			0, #0 disarm 1 arm
			0, #21196 force arm,disarm
			0,0,0,0,0
		)
		vehicle.send_mavlink(_msg)
	


class get_attribute_thread(QThread):
	any_signal=pyqtSignal(tuple)
	def __init__(self,parent=None):
		super(get_attribute_thread,self).__init__(parent)

	def run(self):
		while True:
			global vehicle
			if vehicle is not None:
				package=(vehicle.mode.name,
						vehicle.armed,
						int(vehicle.location.global_relative_frame.alt),
						float(vehicle.airspeed),
						float(vehicle.groundspeed),
						(vehicle.battery.level),
						float(vehicle.attitude.roll*180/math.pi),
						float(vehicle.attitude.pitch*180/math.pi),
						float(vehicle.attitude.yaw*180/math.pi)
						)
				self.any_signal.emit(package)
			time.sleep(1)

	def stop(self):
		package=('','')
		self.any_signal.emit(package)
		self.terminate()

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
		t0 = math.cos(math.radians(yaw * 0.5))
		t1 = math.sin(math.radians(yaw * 0.5))
		t2 = math.cos(math.radians(roll * 0.5))
		t3 = math.sin(math.radians(roll * 0.5))
		t4 = math.cos(math.radians(pitch * 0.5))
		t5 = math.sin(math.radians(pitch * 0.5))

		w = t0 * t2 * t4 + t1 * t3 * t5
		x = t0 * t3 * t4 - t1 * t2 * t5
		y = t0 * t2 * t5 + t1 * t3 * t4
		z = t1 * t2 * t4 - t0 * t3 * t5

		return [w, x, y, z]

x=int
y=int
def method(xc,yc):
	global x
	global y
	x=xc
	y=yc

class get_message(QThread):

	any_signal=pyqtSignal(object)
	def __init__(self,parent=None):
		super(get_message,self).__init__(parent)
		global vehicle
		self.uav=vehicle
		
	

	def run(self):
		use_yaw_rate=False
		roll_angle=0.0
		pitch_angle=0.0
		yaw_angle=0.0
		thrust=0.5
		yaw_rate=0.0
		global x
		global y
		while True:
			print(x,y)
			if self.uav is not None:
				donus_acisi=40
				roll_angle=((x-512)/512)*donus_acisi
				pitch_angle=((y-384)/384)*-donus_acisi
				_msg = self.uav.message_factory.set_attitude_target_encode(
					0,  # time_boot_ms
					1,  # Target system
					1,  # Target component
					0b00000000 if use_yaw_rate else 0b00000100,
					to_quaternion(roll_angle, pitch_angle,
								yaw_angle),  # Quaternion
					0,  # Body roll rate in radian
					0,  # Body pitch rate in radian
					math.radians(yaw_rate),  # Body yaw rate in radian/second
					thrust  # Thrust
				)
				self.any_signal.emit(_msg)
				
			time.sleep(0.1)

	def stop(self):
		self.terminate()


