import dronekit
import time
from PyQt5.QtCore import pyqtSignal, QThread
import math
import cv2
import numpy as np
from ultralytics import YOLO
from PyQt5.QtGui import QImage,QPixmap
import torch

import subprocess as sp
import numpy as np
import cv2



class KameraAl_pc(QThread):
	any_signal=pyqtSignal(object)
	def __init__(self,parent=None):
		super(KameraAl_pc,self).__init__(parent)
		self.cap=cv2.VideoCapture(0)

	def run(self):
		while True:
			
			ret,frame_local=self.cap.read()
			if ret:
				frame_local=cv2.resize(frame_local,(1024,768))
				# frame_local=cv2.cvtColor(frame_local,cv2.COLOR_BGR2RGB)
				# k=cv2.waitKey(1)
				self.any_signal.emit(frame_local)
			else:
				self.any_signal.emit(None)
	def stop(self):
		self.terminate()


class KameraAl_Herelink(QThread):
	any_signal=pyqtSignal(object)
	def __init__(self,parent=None):
		super(KameraAl_Herelink,self).__init__(parent)
		FFMPEG_BIN = "ffmpeg"

		command = [ FFMPEG_BIN,
		            '-i', 'rtsp://192.168.42.129:8554/fpv_stream',
		            '-f', 'image2pipe',
		            '-pix_fmt', 'rgb24',
		            '-vcodec', 'rawvideo', '-']

		self.pipe = sp.Popen(command, stdout = sp.PIPE, bufsize=10**8)


	def run(self):
		while True:

		
			raw_image = self.pipe.stdout.read(1920*1080*3)
			image = np.frombuffer(raw_image, dtype='uint8')

			if image is None:
				self.any_signal.emit(frame_local)
				print("here_cam_none")
				continue
			image = image.reshape((1080,1920,3))
			frame_local=cv2.resize(image,(1024,768))
			frame_local=cv2.cvtColor(frame_local,cv2.COLOR_BGR2RGB)
			# print("*"*100, frame_local)
			self.any_signal.emit(frame_local)

	def stop(self):
		self.terminate()
		

