import pybullet as p  # PyBullet simulator
import numpy as np  # Numpy library
import cv2
import os
import matplotlib.pyplot as plt
from os.path import isfile, join


class cameraTool:
    def __init__(self):

        self.recordVideo = False  
       
        self.fps = 24
        self.heightImage = 512
        self.widthImage = 512
        self.timeBeginning = 0 #time (s) the record begin
        self.nbImage = 0 #current nb of image save

        self.counterImage = 0
        self.counterVideo = 0
        
        # Parameters for the position of the camera
        self.fov = 45
        self.aspect = self.widthImage / self.heightImage
        self.near = 2
        self.far = 10

        self.cameraTargetPosition = [0.25,0.25,0] 
        self.distance = 3.
        self.yaw  = 30.
        self.pitch = -35.
        self.roll = 0.
        self.upAxisIndex = 2
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(self.cameraTargetPosition,self.distance,self.yaw,self.pitch,self.roll,self.upAxisIndex)
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.aspect, self.near, self.far)

        self.adressImage = 'isae/dataVideo/images/'
        self.adressVideo = 'isae/dataVideo/videos/'

    def getRGBAImage(self):
        # Get depth values using the OpenGL renderer
        images = p.getCameraImage(self.widthImage,
                                self.heightImage,
                                self.view_matrix,
                                self.projection_matrix,
                                renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgba_opengl = np.reshape(images[2], (self.heightImage, self.widthImage, 4))

        #########################
        # Not using .png, but conversion RGBA to BGR(used by openCV)
        # Bad image rendering
        #########################
        #rgb = rgba_opengl[:,:,0:3]
        #alpha = rgba_opengl[:,:,3]/255.
        #rgb[:,:,0] = rgba_opengl[:,:,2]*alpha +(np.ones(height)-alpha)*255.
        #rgb[:,:,1] = rgba_opengl[:,:,1]*alpha +(np.ones(height)-alpha)*255.
        #rgb[:,:,2] = rgba_opengl[:,:,0]*alpha +(np.ones(height)-alpha)*255.
        return rgba_opengl

    def saveImage(self,rgbaFrame):
        linkImg = self.adressImage + 'image_' +  str(self.counterImage) + '.png'
        self.counterImage += 1
        plt.imsave(linkImg, rgbaFrame)

    def saveVideo(self):
        frame_array = []
        files = [f for f in os.listdir(self.adressImage) if isfile(join(self.adressImage, f))]

        #for sorting the file names properly
        files.sort(key = lambda x: int(x[len('image_'):len(x)-4]))

        for i in range(len(files)):
            filename=self.adressImage + files[i]
            #reading each files
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width,height)
            #inserting the frames into an image array
            frame_array.append(img)
        
        adressVideo = self.adressVideo + 'marche_' + str(self.counterVideo) + '.avi'

        out = cv2.VideoWriter(adressVideo,cv2.VideoWriter_fourcc(*'DIVX'), self.fps, size) 


        for i in range(len(frame_array)):
            # writing to a image array
            out.write(frame_array[i])
        out.release()