import math
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import cv2
from arayuz_python import Ui_MainWindow
import dronekit
import time
from dronekit_scripts import VehicleData
from dronekit_scripts import get_attribute_thread
from dronekit_scripts import get_message,method
from dronekit_scripts import to_quaternion
from Camera_take import KameraAl_pc
from Camera_take import KameraAl_Herelink
import threading
from ultralytics import YOLO
import torch
import numpy as np
from copy import deepcopy
from find_gps import findGPS

class arayuz_form(QMainWindow):	
	def __init__(self):
		super().__init__()
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		self.Buton_Bagla()
		self.uavData=VehicleData()
		self.Frame=None
		self.x,self.y=0,0
		self.gps=None
		self.kamera_thread=None
		self.kilit=False
		self.model_name='best_75.pt'
		self.model2_name='976.pt' 
		self.model = YOLO(self.model_name)
		self.model2=YOLO(self.model2_name)
		#------değişenler-------
		# self.model_name='976.pt'
		self.arac=0 #0 arac 1 ucak
		self.conf=0.7
		self.Camera_resources=0#0 kamera 1 rtsp --------------------------
		self.ui.l_kamera_alt.setText('1000')
		self.ui.l_kamera_ust.setText('2000')
		self.connection_string="udpout:192.168.42.129:14552"
		# self.connection_string="tcp:127.0.0.1:5762"
		

	def baglan(self):
		self.uavData.connectVehicle(self.connection_string)
		self.ui.l_baglanti.setText("Baglandi")
		self.ui.l_baglanti.setStyleSheet('color: rgb(0, 255, 0)')
		self.ui.b_baglan.setEnabled(False)

		self.attribute_thread=get_attribute_thread()
		self.attribute_thread.any_signal.connect(self.attribute_bastir)
		self.attribute_thread.start()

	def baglantiyi_kes(self):
		# self.attribute_thread.stop()
		self.uavData.disconnectVehicle()
		self.ui.l_baglanti.setText("baglanti yok")
		self.ui.l_baglanti.setStyleSheet('color: rgb(255, 0, 0)')
		self.attribute_thread.stop()
		self.ui.b_baglan.setEnabled(True)
	
	def attribute_bastir(self,package):
		self.ui.l_mod.setText(str(package[0]))
		self.ui.l_alt.setText(str(package[2]))
		self.ui.l_airspeed.setText(str(package[3]))
		self.ui.l_groundspeed.setText(str(package[4]))
		self.ui.l_battery.setText(str(package[5]))
		self.ui.l_roll.setText(str(package[6]))
		self.ui.l_pitch.setText(str(package[7]))
		self.ui.l_yaw.setText(str(package[8]))

		if package[1]==True:
			self.ui.l_arm.setText('Arm')
			self.ui.l_arm.setStyleSheet('color:rgb(0,255,0)')
		elif package[1]==False:
			self.ui.l_arm.setText('Disarm')
			self.ui.l_arm.setStyleSheet('color:rgb(255,0,0)')	

	def mesaj_al(self):
		self.message_thread=get_message()
		self.message_thread.any_signal.connect(self.message_gonder)
		self.message_thread.start()
		self.ui.b_takipBaslat.setEnabled(False)
		self.uavData.mod_degistir('GUIDED')
		
	
	def mesaji_kes(self):
		if self.message_thread is not None:
			self.message_thread.stop()
		self.ui.b_takipBaslat.setEnabled(True)
		self.uavData.mod_degistir('AUTO')


	def message_gonder(self,msg):
		self.uavData.uav.send_mavlink(msg)

	"""Commands: 
	16 waypoint
	21 land
	22 takeoff
	177 do_jump
	(cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)"""

	def komut_indir(self): 
		if self.uavData.uav is not None:
			print("-----silmeden önce------")
			cmds=self.uavData.uav.commands
			cmds.download()
			cmds.wait_ready()
			mission_list=[]

			for i in cmds:
				print(i)
			
			for cmd in cmds:
				if cmd.command==177: #do_jump
					jump_wp=int(cmd.param1) # silinecek = jump_wp-1 tane
				
			for cmd in cmds: #takeoff ve yükseklik az olanları sil
				if cmd.seq<jump_wp:
					pass
				else:
					mission_list.append(cmd)
			
			for cmd in mission_list:
				if cmd.command==177:
					if cmd.param1==1.0:
						pass
					else:
						cmd.param1-=jump_wp-1
			next_wp=cmds.next
			cmds.clear()
			for i in mission_list:
				cmds.add(i)
			# cmds.next=next_wp
			
			self.uavData.uav.mode='LOITER'
			time.sleep(0.5)
			cmds.upload()
			self.uavData.uav.mode='AUTO'


			# print("next: ",cmds.next)

	def frame_durdur(self):
		self.kamera_thread.stop()

	def Frame_baslat(self):
		if self.Camera_resources==0:
			if self.kamera_thread is None:
				self.kamera_thread=KameraAl_pc()
				self.kamera_thread.any_signal.connect(self.frame_bas)
				self.kamera_thread.start()
				time.sleep(2)
				threading.Thread(target=self.read_frame).start()
			else:
				self.kamera_thread.stop()
				# self.kamera_thread=KameraAl_pc()
				# self.kamera_thread.any_signal.connect(self.frame_bas)
				self.kamera_thread.start()
		else:
			if self.kamera_thread is None:
				self.kamera_thread=KameraAl_Herelink()
				self.kamera_thread.any_signal.connect(self.frame_bas)
				self.kamera_thread.start()
				time.sleep(2)

				threading.Thread(target=self.read_frame).start()
			else:
				self.kamera_thread.stop()
				self.kamera_thread=KameraAl_Herelink()
				self.kamera_thread.any_signal.connect(self.frame_bas)
				self.kamera_thread.start()

	def frame_bas(self,frame):
		self.Frame=frame

	
	def score_frame(self, frame,conf):
		if self.arac==0: #araç
			results = self.model(frame,conf=conf,verbose=False)
		elif self.arac==1: #uçak
			results=self.model2(frame,conf=conf,verbose=False)
		# print(len(results))
		for result in results:
			result_return=result.boxes.xyxyn
			if(len(result.boxes.cls)>0):
				# print(f'{result.boxes.cls[0]}')
				a=result.boxes.cls[0]
			else:
				# print("tespit yok")
				a=-1

		return result_return,a
	
	def plot_boxes(self, results, frame, labels):
		x_shape, y_shape = frame.shape[1], frame.shape[0]
		for row in results:
			x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
			bgr = (0, 0, 255)
			cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
			x = int((x1+x2)/2)
			y = int((y1+y2)/2)
			method(x,y)
			if labels!=-1:
				if self.arac==0:
					if self.uavData.uav!=None:
						self.gps=findGPS(self.uavData.uav.location.global_relative_frame.lat,
									self.uavData.uav.location.global_relative_frame.lon,
									self.uavData.uav.location.global_relative_frame.alt,
									y,x,
									768,1024,
									170,
									200,
									self.uavData.uav.attitude.roll,
									self.uavData.uav.attitude.pitch)
			if self.kilit==False:
				# self.ui.l_kilitlenme.setText("KİLİTLENİLDİ !")
				# self.ui.l_kilitlenme.setStyleSheet('color: rgb(204, 0, 0);')
				self.kilit=True
				
			cv2.line(frame,(512,384),(x,y),(255,255,255),2)			
		return frame
	
	def read_frame(self):
		
		self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

		print("Using Device: ", self.device)

		while True:
			conf=self.conf
			frame=deepcopy(self.Frame)
			if frame is None:
				# print("Frame yok")
				continue	
				
			else:
				frame = cv2.resize(frame, (1024, 768))
				
				results,labels = self.score_frame(frame,self.conf)

				results,labels = self.score_frame(frame,conf)
				frame = self.plot_boxes(results, frame, labels)
				frame = cv2.resize(frame, (1024, 768))
				frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
				k = cv2.waitKey(1)
				# cv2.imshow("frame",frame)
				
				height, width, channel = frame.shape
				step = channel * width
				qImg = QImage(frame.data, width, height,step, QImage.Format_RGB888)
				self.ui.label_kamera.setPixmap(QPixmap.fromImage(qImg))
			time.sleep(0.01)
		

	def kamera_indir(self):
		_msg=self.uavData.uav.message_factory.command_long_encode(
			0,0,
			dronekit.mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
			0,
			5,
			int(self.ui.l_kamera_alt.text()), 
			0,0,0,0,0
		)
		self.uavData.uav.send_mavlink(_msg)

	def kamera_kaldir(self):
		_msg=self.uavData.uav.message_factory.command_long_encode(
			0,0,
			dronekit.mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
			0,
			5,
			int(self.ui.l_kamera_ust.text()), 
			0,0,0,0,0
		)
		self.uavData.uav.send_mavlink(_msg)

	def arac_degistir(self):
		if self.arac==1:
			self.arac=0  
		else:
			self.arac=1
		self.ui.l_arac.setText('kara' if self.arac==0 else 'hava')
	
	def conf_degistir(self):
		self.conf=float(self.ui.l_conf.text())

	def keyPressEvent(self, e):
		if e.key()==16777249:
			self.ui.l_kilitlenme.setText("")
			self.ui.l_kilitlenme.setStyleSheet('background-color: rgb(220, 175, 100);')
			self.kilit=False

	def kacis(self,yon):
		if yon=='sagalt':
			roll,pitch=25,-25
		elif yon=='sagust':
			roll,pitch=25,25
		elif yon=='solalt':
			roll,pitch=-25,-25
		else: #solust
			roll,pitch=-25,25

		_msg = self.uavData.uav.message_factory.set_attitude_target_encode(
			0,  # time_boot_ms
			1,  # Target system
			1,  # Target component
			0b00000000 if 1==0 else 0b00000100,
			to_quaternion(roll, pitch, 0),  # Quaternion
			0,  # Body roll rate in radian
			0,  # Body pitch rate in radian
			math.radians(0),  # Body yaw rate in radian/second
			0.5  # Thrust
		)
		self.uavData.uav.send_mavlink(_msg)
			

	def Buton_Bagla(self):
		self.ui.b_baglan.clicked.connect(self.baglan)
		self.ui.b_baglanti_kes.clicked.connect(self.baglantiyi_kes)
		self.ui.b_modAUTO.clicked.connect(lambda:self.uavData.mod_degistir('AUTO'))
		self.ui.b_modFBWA.clicked.connect(lambda:self.uavData.mod_degistir('FBWA'))
		self.ui.b_modLOITER.clicked.connect(lambda:self.uavData.mod_degistir('LOITER'))
		self.ui.b_modRTL.clicked.connect(lambda:self.uavData.mod_degistir('RTL'))
		self.ui.b_modUYGULA.clicked.connect(lambda:self.uavData.mod_degistir(self.ui.comboBox.currentText()))
		self.ui.b_armET.clicked.connect(VehicleData.arm_et)
		self.ui.b_disarmET.clicked.connect(VehicleData.disarm_et)
		self.ui.b_takipBaslat.clicked.connect(self.mesaj_al)
		self.ui.b_takipDurdur.clicked.connect(self.mesaji_kes)
		self.ui.pushButton.clicked.connect(self.komut_indir)
		self.ui.b_start_kamera.clicked.connect(self.Frame_baslat)
		self.ui.b_stop_kamera.clicked.connect(self.frame_durdur)
		self.ui.b_kamera_indir.clicked.connect(self.kamera_indir)
		self.ui.b_kamera_kaldir.clicked.connect(self.kamera_kaldir)
		self.ui.b_arac_degistir.clicked.connect(self.arac_degistir)
		self.ui.b_conf_ayarla.clicked.connect(self.conf_degistir)
	
	cv2.destroyAllWindows()

uygulama = QApplication([])
pencere = arayuz_form()
# -----------------------------------------------------#

pencere.setWindowTitle("MERKUT_MGM")

# -----------------------------------------------------#

pencere.show()
uygulama.exec_()
