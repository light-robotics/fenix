import numpy as np
import ArducamDepthCamera as ac

MAX_DISTANCE = 4

def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame

def process_depth(depth_buf: np.ndarray) -> np.ndarray:
    depth_buf = np.nan_to_num(depth_buf)

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)

    return depth_buf


if __name__ == "__main__":
    cam = ac.ArducamCamera()
    if cam.open(ac.TOFConnect.CSI,0) != 0 :
        print("initialization failed")
    if cam.start(ac.TOFOutput.DEPTH) != 0 :
        print("Failed to start camera")
    cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE)

    frame = cam.requestFrame(200)
    if frame != None:
        depth_buf = frame.getDepthData()
        amplitude_buf = frame.getAmplitudeData()
        cam.releaseFrame(frame)
        #amplitude_buf*=(255/1024)
        #amplitude_buf = np.clip(amplitude_buf, 0, 255)

    #result = amplitude_buf.astype(np.uint8)
    #result = process_depth(depth_buf)
    result_image = process_frame(depth_buf, amplitude_buf)
    result = depth_buf
    print(type(result), result, len(result), len(result[0]), min(result[0]), max(result[0]))
    cam.stop()
    cam.close()
    #np.savetxt('/fenix/fenix/wrk/depth_buf', result)
    np.savetxt('/fenix/fenix/wrk/result_image', result_image)
