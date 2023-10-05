import cv2
import time
import threading
import os

from .localization.localization import localization
from .marker_detection.marker_detection import marker_detection


class camera():
    def __init__(self, parent, _ZR30_IP:str="192.168.144.25", _ZR30_PORT:int=37260):
        self.parent = parent

        # Init the localization class
        self.localization =\
            localization(image_fx=736.9205, image_fy=738.6126,  # No idea where these are from but they seem correct
                         image_height=720, image_width=1280,
                         marker_size=0.4,  # The distance from the center of the marker to the ground times 2...
                         detection_class=marker_detection)  # The class which provides the detection of the marker

        # For control of the gimbal, but is not really used right now
        # self.camera = SIYISDK(server_ip=_ZR30_IP, port=_ZR30_PORT)
        # self.camera.connect()

        self.init_variables()
        
        self.capture = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 0)  # This doesnt really seem to help but idk


        ## NBNBNB Using the Ros2 create_timer function ends poorly when using the camera and opencv
        # Don't repeat this mistake
        threading.Thread(target=lambda: self.capture_image_thread(), daemon=True).start()
        

    def init_variables(self):
        # The old distance estimate
        self.old_dist = 0

        self.start_image_loop = False

        # The position estimate
        self.x = None
        self.y = None
        self.z = None

        self.x_raw = None
        self.y_raw = None
        self.z_raw = None

        self.localization_calculation_flag = False
        self.image_number = 0

        # NBNB TODO Keep in mind that it will save all images, which is kind of a lot
        # So keep this flag in mind
        self.save_images = True
        
        raw_image_folder = 'images'
        analyzed_image_folder = 'analyzed_images'

        f = open(raw_image_folder + '/image_cnt.txt', 'rw')
        self.image_folder_cnt = int(f.read())

        self.total_raw_image_folder = raw_image_folder + '/{:04}'.format(self.image_cnt)
        if not os.path.exists(self.total_raw_image_folder):
            os.makedirs(self.total_raw_image_folder)

        self.total_analyzed_image_folder = analyzed_image_folder + '/{:04}'.format(self.image_cnt)
        if not os.path.exists(self.total_analyzed_image_folder):
            os.makedirs(self.total_analyzed_image_folder)
        

        f.write(str(self.image_folder_cnt + 1))

    def save_image_thread(self, image, folder_name='images'):
        if self.save_images:
            cv2.imwrite(folder_name + '/{:06}.png'.format(self.image_number),image)

    def capture_image_thread(self):
        while True:
            self.capture_image()

    def capture_image(self):
        # self.capture.grab()  # This is to make sure , isthe buffer gets emptied
        _, image = self.capture.read()

        if image is None:
            print('No image. Is the camera on?')
            return
        
        if self.parent.odometry.z is None:
            return
    
        # The estimate does not work at very low heights
        # And the range finder also doesn't kek
        if abs(self.parent.odometry.z) < 1.0:
            return    
        
        if not self.start_image_loop:
            return
        
        threading.Thread(target=lambda: self.save_image_thread(image.copy(), self.total_raw_image_folder), daemon=True).start()
        self.image_number += 1
        
        # This is to make sure the images doesnt fall behind 
        threading.Thread(target=lambda: self.localization_loop(image.copy()), daemon=True).start()

    def localization_loop(self, image):
        
        if self.localization_calculation_flag:
            return
        self.localization_calculation_flag = True

        # Ensure that z is positive
        z = abs(self.parent.odometry.z)

        x, y, p_x, p_y = self.localization.update(image,
                                                  time.time(),                  # Current time
                                                  z,                            # Current height
                                                  0, 0, 0,                      # Gimbal attitude
                                                  self.parent.odometry.v_x, self.parent.odometry.v_y) # Current velocity
        

        if x is None:
            self.localization_calculation_flag = False
            print('find marker')

        self.x = x
        self.y = y
        self.z = z
        
        self.x_raw, self.y_raw = self.localization.get_raw_estimate()
        self.z_raw = z
        
        self.localization_calculation_flag = False
        threading.Thread(target=lambda: self.save_image_thread(image.copy(), self.total_analyzed_image_folder), daemon=True).start()
        return

        # For putting in the position of the gimbal
        # Only the yaw actually becomes something else than 0 though
        # -np.deg2rad(self.parent.gimbal_attitude.roll),
        # np.deg2rad(self.parent.gimbal_attitude.pitch),
        # np.deg2rad(self.parent.gimbal_attitude.yaw),#rz + 



