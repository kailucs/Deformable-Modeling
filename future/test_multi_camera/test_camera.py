## setup logging
#import logging
#logging.basicConfig(level = logging.INFO)

## import the package
import pyrealsense as pyrs

def run_camera(device_id,fps,serv):
    ## start the service - also available as context manager
    #serv = pyrs.Service()
    serv = serv
    ## create a device from device id and streams of interest
    cam = serv.Device(device_id = device_id, streams = [pyrs.stream.ColorStream(fps = fps)])

    ## retrieve 60 frames of data
    for _ in range(1000):
        cam.wait_for_frames()
        print(device_id)
        #print(cam.color)

    ## stop camera and service
    cam.stop()
    serv.stop()
