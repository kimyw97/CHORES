import time
import ArducamDepthCamera as ac
from ArducamDepthCamera.ArducamDepthCamera import FrameType, Connection

tof = ac.ArducamCamera()

if tof.open(Connection.CSI, 0) != 0:
    print("❌ 카메라 열기 실패")
    exit()

if tof.start(FrameType.DEPTH) != 0:
    print("❌ 스트림 시작 실패")
    exit()

time.sleep(2)  # 스트리밍 안정화 시간

for i in range(10):
    frame = tof.requestFrame(FrameType.DEPTH)
    if frame:
        print(f"[{i}] ✅ frame OK - shape: {frame.depth_data.shape}")
    else:
        print(f"[{i}] ❌ frame is None")
    time.sleep(0.1)

tof.stop()
tof.close()
