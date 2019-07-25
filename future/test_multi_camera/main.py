import test_camera
import multiprocessing
import pyrealsense as pyrs

def main():
    serv = pyrs.Service()
    p1 = multiprocessing.Process(target=test_camera.run_camera,args=(0,60,serv))
    p2 = multiprocessing.Process(target=test_camera.run_camera,args=(1,60,serv))
    p1.start()
    p2.start()

if __name__ == "__main__":
    main()
