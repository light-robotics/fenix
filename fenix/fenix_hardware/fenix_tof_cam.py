import sys
import numpy as np
import ArducamDepthCamera as ADC

class FenixTofCamera():
    MAX_DISTANCE = 4

    def __init__(self):
        self.cam = ADC.ArducamCamera()
        if self.cam.open(ADC.TOFConnect.CSI,0) != 0:
            print("initialization failed")
        if self.cam.start(ADC.TOFOutput.DEPTH) != 0:
            print("Failed to start camera")
        self.cam.setControl(ADC.TOFControl.RANG, self.MAX_DISTANCE)
    
    def process_depth(self, depth_buf: np.ndarray) -> np.ndarray:
        depth_buf = np.nan_to_num(depth_buf)

        depth_buf = (1 - (depth_buf/self.MAX_DISTANCE)) * 255
        depth_buf = np.clip(depth_buf, 0, 255)

        return depth_buf

    def read_depth(self, height=0):
        frame = self.cam.requestFrame(200)
        if frame != None:
            depth_buf = frame.getDepthData()
            self.cam.releaseFrame(frame)
            np.savetxt(f'/fenix/fenix/wrk/tof_data/depth_{height}', depth_buf)

    def __del__(self):
        self.cam.stop()
        self.cam.close()

if __name__ == "__main__":
    ftc = FenixTofCamera()
    ftc.read_depth()
