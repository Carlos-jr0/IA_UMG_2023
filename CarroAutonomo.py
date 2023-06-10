import io
import socket
import struct
import time
import picamera
import sys
import cv2
import numpy as np
import RPi.GPIO as GPIO

from socket import *

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.1.100', 8000))
connection = client_socket.makefile('wb')

# Color detection thresholds
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])
lower_green = np.array([50, 100, 100])
upper_green = np.array([70, 255, 255])

def measure():
    """
    Measure distance and detect color
    """
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    start = time.time()

    while GPIO.input(GPIO_ECHO) == 0:
        start = time.time()

    while GPIO.input(GPIO_ECHO) == 1:
        stop = time.time()

    elapsed = stop - start
    distance = (elapsed * 34300) / 2

    # Capture a frame from the camera
    ret, frame = camera.read()

    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only the red and green regions
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Count the number of red and green pixels in the masks
    red_pixel_count = cv2.countNonZero(mask_red)
    green_pixel_count = cv2.countNonZero(mask_green)

    # Check if red light is detected
    if red_pixel_count > 0:
        print("Red light detected. Stopping.")
        # Add your code to stop the movement here
        GPIO.output(MOTOR_PIN_1, GPIO.LOW)
        GPIO.output(MOTOR_PIN_2, GPIO.LOW)

    # Check if green light is detected
    elif green_pixel_count > 0:
        print("Green light detected. Advancing.")
        # Add your code to move forward here
        GPIO.output(MOTOR_PIN_1, GPIO.HIGH)
        GPIO.output(MOTOR_PIN_2, GPIO.LOW)

    return distance


GPIO.setwarnings(False)

# create a socket and bind socket to the host
client_socket = socket(AF_INET, SOCK_STREAM)
client_socket.connect(('192.168.1.100', 8002))

# referring to the pins by GPIO numbers
GPIO.setmode(GPIO.BCM)

# define pi GPIO
GPIO_TRIGGER = 23
GPIO_ECHO = 24
MOTOR_PIN_1 = 25  # Example GPIO pin for motor control
MOTOR_PIN_2 = 26  # Example GPIO pin for motor control

# output pin: Trigger
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
# input pin: Echo
GPIO.setup(GPIO_ECHO, GPIO.IN)
# initialize trigger pin to low
GPIO.output(GPIO_TRIGGER, False)

try:
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)      # pi camera resolution
        camera.framerate = 15               # 15 frames/sec
        time.sleep(2)                       # give 2 secs for camera to initialize
        start = time.time()
        stream = io.BytesIO()

        # send jpeg format video stream
        for foo in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
            connection.write(struct.pack('<L', stream.tell()))
            connection.flush()
            stream.seek(0)
            connection.write(stream.read())
            if time.time() - start > 600:
                break
            stream.seek(0)
            stream.truncate()

    connection.write(struct.pack('<L', 0))

    try:
        while True:
            distance = measure()
            print("Distance: %.1f cm" % distance)
            # send data to the host every 0.5 sec
            client_socket.send(str(distance).encode('utf-8'))
            time.sleep(0.5)

    finally:
        finish = time.time()
        print('Sent %d images in %d seconds at %.2f fps' % (
            output.count, finish - start, output.count / (finish - start)))
        connection.close()
        client_socket.close()
        connection.close()
        client_socket.close()
        client_socket.close()
        GPIO.cleanup()
