import sys
import numpy as np
import ArducamDepthCamera as ac

if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.open(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.RAW) != 0 :
        print("Failed to start camera")

    frame = cam.requestFrame(200)
    print(type(frame), frame)
    if frame != None:
        buf = frame.getRawData()
        print(len(buf), len(buf[0]), min(buf[0]), max(buf[0]))
        cam.releaseFrame(frame)
    cam.stop()
    cam.close()
