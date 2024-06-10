import subprocess as sp
import numpy as np
import cv2

FFMPEG_BIN = "ffmpeg"

command = [ FFMPEG_BIN,
            '-i', 'rtsp://192.168.42.129:8554/fpv_stream',
            '-f', 'image2pipe',
            '-pix_fmt', 'rgb24',
            '-vcodec', 'rawvideo', '-']

pipe = sp.Popen(command, stdout = sp.PIPE, bufsize=10**8)

while True:
    raw_image = pipe.stdout.read(1920*1080*3)
    image = np.frombuffer(raw_image, dtype='uint8')
    image = image.reshape((1080,1920,3))
    print(image)
    cv2.imshow('image', image)
    cv2.waitKey(1)
