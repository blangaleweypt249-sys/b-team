import cv2
import numpy as np
import yaml

# 修改为你的棋盘格内角点、方格尺寸(m)
pattern_size = (6,8)
square_size = 0.025

cap = cv2.VideoCapture("/dev/video2")
objp = np.zeros((np.prod(pattern_size),3), np.float32)
objp[:,:2] = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)*square_size

obj_points = []
img_points = []
gray = None

print("采集棋盘格画面，按s保存，按q结束开始标定")
while True:
    ret, frame = cap.read()
    if not ret: break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, pattern_size)
    disp = frame.copy()
    if found:
        cv2.drawChessboardCorners(disp, pattern_size, corners, found)
    cv2.imshow("view", disp)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('s') and found:
        obj_points.append(objp)
        img_points.append(corners)
        print(f"已采集 {len(img_points)} 组")
    if k == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

if gray is None:
    print("未读取到图像，标定终止")
else:
    print("开始计算参数...")
    h,w = gray.shape
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w,h), None, None)
    print(f"重投影误差: {ret}")

    cam_data = {
        "camera_name": "camera",
        "image_width": w,
        "image_height": h,
        "camera_matrix": {"data": mtx.flatten().tolist()},
        "distortion_coefficients": {"data": dist.flatten().tolist()},
        "distortion_model": "plumb_bob",
        "rectification_matrix": {"data": np.eye(3).flatten().tolist()},
        "projection_matrix": {"data": np.hstack([mtx, np.zeros((3,1))]).flatten().tolist()}
    }
    with open("camera.yaml","w") as f:
        yaml.dump(cam_data,f)
    print("已保存 camera.yaml")