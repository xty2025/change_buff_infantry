import cv2
import numpy as np
import glob
import os

def camera_calibration(image_dir="../config", pattern_size=(9, 6), square_size=1.0):
    """
    使用棋盘图进行相机标定
    :param image_dir: 图像文件夹路径
    :param pattern_size: 棋盘格内角点数量 (cols, rows)
    :param square_size: 每个棋盘格的物理尺寸
    :return: 相机矩阵、畸变系数、旋转向量、平移向量、重投影误差
    """

    # 世界坐标系下的三维点 (0,0,0), (1,0,0) ...
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # 存储所有图像的 objpoints 和 imgpoints
    objpoints = []
    imgpoints = []

    # 获取所有 jpg/png 图片
    images = glob.glob(os.path.join(image_dir, "*.jpg"))
    images += glob.glob(os.path.join(image_dir, "*.png"))

    if len(images) == 0:
        print("❌ 在目录中没有找到图片：", image_dir)
        return

    print("共找到图片数量：", len(images))

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 查找棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            objpoints.append(objp)
            # 亚像素优化
            criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            print(f"✔ 检测成功: {fname}")
        else:
            print(f"✖ 检测失败: {fname}")

    # 执行标定
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    # 计算重投影误差
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i],
                                          camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error

    reprojection_error = total_error / len(objpoints)

    # 输出结果
    print("\n================ 标定完成 ================\n")
    print("相机内参矩阵 Camera Matrix:\n", camera_matrix)
    print("\n畸变系数 Distortion Coeffs:\n", dist_coeffs.ravel())
    print("\n重投影误差 Reprojection Error:\n", reprojection_error)

    return camera_matrix, dist_coeffs, rvecs, tvecs, reprojection_error


if __name__ == "__main__":
    camera_calibration()