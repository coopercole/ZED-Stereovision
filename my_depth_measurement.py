import pyzed.sl as sl
import math
import numpy as np
import sys
import math
import cv2

def getMousePos(image):
    def onMouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            param['x'] = x
            param['y'] = y
            param['event'] = event

    param = {'x': -1, 'y': -1, 'event': -1}
    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", onMouse, param)
    cv2.imshow("Image", image)

    while param['event'] != cv2.EVENT_LBUTTONDOWN:
        cv2.waitKey(10)
    
    cv2.destroyWindow("Image")

    return param['x'], param['y']


# Create a Camera object
zed = sl.Camera()

# Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)

# Open the camera
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
    print("Camera Open : "+repr(status)+". Exit program.")
    exit()

# Create and set RuntimeParameters after opening the camera
runtime_parameters = sl.RuntimeParameters()

i = 0
image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()

mirror_ref = sl.Transform()
mirror_ref.set_translation(sl.Translation(2.75,4.0,0))

if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
    # Retrieve left image
    zed.retrieve_image(image, sl.VIEW.LEFT)
    # Retrieve depth map. Depth is aligned on the left image
    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
    # retireve the point cloud
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)


    # Get the x, y values by picking a pixel in the image frame with my mouse
    # x, y = getMousePos(image.get_data())
    x = 630
    y = 540

    print(f"Pixel coordinates: {{{x};{y}}}")

    # retrieve the point cloud value at the center of the image
    point_cloud_value = point_cloud.get_value(x, y)
    print(f"Point cloud value at {{{x};{y}}}: {point_cloud_value}")
    print(f"Depth value at {{{x};{y}}}: {depth.get_value(x, y)}")
    print(f"Point Cloud Value at index 2 {point_cloud_value[2]}")

    if math.isfinite(point_cloud_value[2]):
        distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                            point_cloud_value[1] * point_cloud_value[1] +
                            point_cloud_value[2] * point_cloud_value[2])
        print(f"Distance to Camera at {{{x};{y}}}: {distance}")
    else : 
        print(f"The distance can not be computed at {{{x};{y}}}")
           

    # Close the camera
    zed.close()
