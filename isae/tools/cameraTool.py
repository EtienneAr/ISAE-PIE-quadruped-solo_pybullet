import pybullet as p  # PyBullet simulator
import numpy as np  # Numpy library
import cv2
import os
import matplotlib.pyplot as plt
from os.path import isfile, join

width = 512
height = 512   
fps = 24

fov = 45
aspect = width / height
near = 2
far = 10

cameraTargetPosition = [0.25,0.25,0] 
distance = 3.
yaw  = 30.
pitch = -35.
roll = 0.
upAxisIndex = 2
view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition,distance,yaw,pitch,roll,upAxisIndex)
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

adress = 'dataImages/images/image_'
pathIn = 'dataImages/images/'

def getRGBAImage():
    # Get depth values using the OpenGL renderer
    images = p.getCameraImage(width,
							height,
							view_matrix,
							projection_matrix,
							renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba_opengl = np.reshape(images[2], (height, width, 4))

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

def saveImage(rgbaFrame,counter):
    linkImg = adress + str(counter) + '.png'
    plt.imsave(linkImg, rgbaFrame)

def saveVideo(nbVideo):
    frame_array = []
    files = [f for f in os.listdir(pathIn) if isfile(join(pathIn, f))]

    #for sorting the file names properly
    files.sort(key = lambda x: int(x[len('image_'):len(x)-4]))

    for i in range(len(files)):
        filename=pathIn + files[i]
        #reading each files
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        #inserting the frames into an image array
        frame_array.append(img)
    
    adressVideo = './dataImages/videos/' + 'marche_' + str(nbVideo) + '.avi'

    out = cv2.VideoWriter(adressVideo,cv2.VideoWriter_fourcc(*'DIVX'), fps, size) 


    for i in range(len(frame_array)):
        # writing to a image array
        out.write(frame_array[i])
    out.release()