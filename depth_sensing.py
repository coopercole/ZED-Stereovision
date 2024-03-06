import pyzed.sl as sl
import math
import numpy as np
import sys
import math
import cv2
import fractions

MILLIMETERS_TO_INCHES = 0.0393701
FRACTIONAL_PRECISION = 16


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

def mm_to_feet_inches_fractions(mm):
    # Convert mm to inches
    inches = mm * MILLIMETERS_TO_INCHES
    # Calculate feet and remaining inches
    feet = int(inches // 12)
    remaining_inches = inches % 12

    # Convert remaining inches to fractions
    remaining_distance = remaining_inches - int(remaining_inches)

    numerator = round((remaining_distance * FRACTIONAL_PRECISION))
    denominator = FRACTIONAL_PRECISION
    fraction = fractions.Fraction(numerator, denominator) 
    
    # Format the result
    result = f"{feet}' {int(remaining_inches)}\" {fraction}"
    return result


def main():

    print(f"TEST MEASUREMENTS: {mm_to_feet_inches_fractions(15050)}")

    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units (for depth measurements)

    # Create a matrix to store image, depth, point cloud
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    # Create a reference to the mirror (IDK WHAT THIS MEANS)
    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera has opened succesfully
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    # A new image is available if grab() returns SUCCESS
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # Retrieve left image
        zed.retrieve_image(image, sl.VIEW.LEFT)
        # Retrieve depth map. Depth is aligned on the left image
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        # Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        # Get and print distance value in mm at the center of the image
        # We measure the distance camera - object using Euclidean distance
        # x = round(image.get_width() / 2)
        # y = round(image.get_height() / 2)

        # choose a point in the image
        x, y = getMousePos(image.get_data())

        err, point_cloud_value = point_cloud.get_value(x, y)
        # print(f"Point cloud value at {{{x};{y}}}: {point_cloud_value}")
        # print(f"Z Depth value at {{{x};{y}}}: {round(depth.get_value(x, y), 2)} mm")
        print(f"Z Depth value at {{{x};{y}}}: {round(depth.get_value(x, y)[1], 2)} mm")
        print(f"Z Depth value at {{{x};{y}}}: {mm_to_feet_inches_fractions(float(depth.get_value(x, y)[1]))}")
        
        if math.isfinite(point_cloud_value[2]):
            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                point_cloud_value[1] * point_cloud_value[1] +
                                point_cloud_value[2] * point_cloud_value[2])
            
            print(f"Point Cloud Distance to Camera at {{{x};{y}}}: {round(distance, 2)} mm")
            distance_fractional = mm_to_feet_inches_fractions(float(distance))
            print(f"Point Cloud Distance to Camera at {{{x};{y}}}: {distance_fractional} ft")
        else : 
            print(f"The distance can not be computed at {{{x};{y}}}")  
        

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()