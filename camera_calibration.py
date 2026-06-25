import os
import sys
from datetime import datetime

import cv2
import numpy as np


def is_gui_available():
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def find_chessboard_corners(images, board_size, show_results=True):
    image_points = []
    success_count = 0

    for index, image in enumerate(images):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(gray, board_size, flags)

        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

            image_points.append(corners)
            success_count += 1
            print(f"图像 {index + 1}: 成功检测到角点")

            if show_results:
                image_with_corners = image.copy()
                cv2.drawChessboardCorners(image_with_corners, board_size, corners, found)
                display = cv2.resize(image_with_corners, None, fx=0.5, fy=0.5)
                cv2.imshow("Detected Corners", display)
                cv2.waitKey(500)
        else:
            print(f"图像 {index + 1}: 未能检测到角点")

    if show_results:
        cv2.destroyWindow("Detected Corners")

    return success_count, image_points


def generate_object_points(board_size, square_size):
    object_points = []
    for row in range(board_size[1]):
        for col in range(board_size[0]):
            object_points.append([col * square_size, row * square_size, 0.0])
    return np.array(object_points, dtype=np.float32)


def run_calibration(object_points, image_points, image_size):
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=0,
    )
    return rms, camera_matrix, dist_coeffs, rvecs, tvecs


def save_calibration_results(
    filename,
    image_size,
    board_size,
    square_size,
    camera_matrix,
    dist_coeffs,
    rms,
):
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_WRITE)

    if not fs.isOpened():
        raise RuntimeError(f"无法写入输出文件: {filename}")

    fs.write("calibration_time", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("board_width", int(board_size[0]))
    fs.write("board_height", int(board_size[1]))
    fs.write("square_size", float(square_size))
    fs.write("camera_matrix", camera_matrix)
    fs.write("distortion_coefficients", dist_coeffs)
    fs.write("rms_reprojection_error", float(rms))
    fs.release()

    print(f"\n标定结果已保存到: {filename}")


def print_calibration_results(camera_matrix, dist_coeffs, rms):
    dist = dist_coeffs.flatten()

    print("\n========== 标定结果 ==========")
    print("\n相机内参矩阵 (Camera Matrix):")
    print(camera_matrix)

    print("\n畸变系数 (Distortion Coefficients):")
    labels = ["k1", "k2", "p1", "p2", "k3"]
    for idx, label in enumerate(labels):
        value = dist[idx] if idx < len(dist) else 0.0
        print(f"{label}: {value}")

    print(f"\n重投影误差 RMS (Reprojection Error): {rms} 像素")
    print("==============================\n")


def compute_extrinsic_pnp(object_points, image_points, camera_matrix, dist_coeffs):
    """
    基于地面棋盘格，使用PnP算法计算相机的外参（旋转向量和平移向量）

    地面棋盘格定义世界坐标系：棋盘平面为Z=0平面（地面），
    X轴沿棋盘宽度方向，Y轴沿棋盘高度方向，Z轴垂直地面向上。

    参数:
        object_points: 棋盘角点的世界坐标（3D点集，Nx3）
        image_points:  棋盘角点的图像坐标（2D点集，Nx1x2）
        camera_matrix: 相机内参矩阵（3x3）
        dist_coeffs:   畸变系数

    返回:
        rvec: 旋转向量（Rodrigues形式，3x1）
        tvec: 平移向量（3x1，单位与squareSize一致）
        R:    旋转矩阵（3x3）
    """
    # 使用PnP算法求解相机位姿
    # SOLVEPNP_ITERATIVE: 基于Levenberg-Marquardt优化的迭代方法，精度较高
    _, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )

    # 将旋转向量转换为旋转矩阵（便于理解和使用）
    R, _ = cv2.Rodrigues(rvec)

    return rvec, tvec, R


def print_extrinsic_results(rvec, tvec, R):
    """
    打印相机外参结果

    显示旋转向量、旋转矩阵、平移向量，以及相机在世界坐标系中的位置。

    参数:
        rvec: 旋转向量（Rodrigues形式）
        tvec: 平移向量
        R:    旋转矩阵（3x3）
    """
    print("\n========== 相机外参（基于地面棋盘格PnP）==========")

    # 输出旋转向量（Rodrigues形式：旋转轴×旋转角）
    print("\n旋转向量 (Rodrigues形式):")
    print(f"  rx = {rvec[0, 0]:.6f}")
    print(f"  ry = {rvec[1, 0]:.6f}")
    print(f"  rz = {rvec[2, 0]:.6f}")

    # 输出旋转矩阵（世界坐标系 → 相机坐标系）
    print("\n旋转矩阵 R（世界坐标系 → 相机坐标系）:")
    print(R)

    # 输出平移向量（世界坐标系原点在相机坐标系中的坐标）
    print("\n平移向量 t（世界坐标系原点在相机坐标系中的位置，单位：mm）:")
    print(f"  tx = {tvec[0, 0]:.3f} mm")
    print(f"  ty = {tvec[1, 0]:.3f} mm")
    print(f"  tz = {tvec[2, 0]:.3f} mm")

    # 计算相机光心在世界坐标系中的位置: C = -R^T * t
    # R是正交矩阵，其逆矩阵等于其转置
    R_transpose = R.T
    camera_pos = -R_transpose @ tvec.reshape(3, 1)

    print("\n相机在世界坐标系中的位置（光心）:")
    print(f"  Xc = {camera_pos[0, 0]:.3f} mm")
    print(f"  Yc = {camera_pos[1, 0]:.3f} mm")
    print(f"  Zc = {camera_pos[2, 0]:.3f} mm （相机距地面高度）")

    # 从旋转矩阵分解欧拉角（ZYX顺序：偏航Yaw → 俯仰Pitch → 翻滚Roll）
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
    else:
        # 万向节死锁时的退化处理
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(-R[1, 2], R[1, 1])
        roll = 0.0

    # 弧度转角度
    pitch_deg = np.degrees(pitch)
    yaw_deg = np.degrees(yaw)
    roll_deg = np.degrees(roll)

    print("\n相机欧拉角（ZYX顺序）:")
    print(f"  偏航角 Yaw   = {yaw_deg:.3f}°")
    print(f"  俯仰角 Pitch = {pitch_deg:.3f}°")
    print(f"  翻滚角 Roll  = {roll_deg:.3f}°")

    print("===================================================\n")


def save_undistortion_results(images, image_paths, camera_matrix, dist_coeffs, output_dir):
    if not images:
        return

    os.makedirs(output_dir, exist_ok=True)

    saved_count = 0
    for index, image in enumerate(images):
        undistorted = cv2.undistort(image, camera_matrix, dist_coeffs)
        comparison = cv2.hconcat([image, undistorted])

        scale = 800.0 / comparison.shape[1] if comparison.shape[1] > 800 else 1.0
        display = cv2.resize(comparison, None, fx=scale, fy=scale)

        cv2.putText(
            display,
            "Original",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            display,
            "Undistorted",
            (display.shape[1] // 2 + 10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        src_name = os.path.basename(image_paths[index]) if index < len(image_paths) else f"image_{index + 1:02d}.jpg"
        stem, _ext = os.path.splitext(src_name)
        out_path = os.path.join(output_dir, f"{stem}_undistort_compare.jpg")
        if cv2.imwrite(out_path, display):
            saved_count += 1

    print(f"\n去畸变对比图已保存到: {output_dir}")
    print(f"成功保存 {saved_count} / {len(images)} 张图像")


def collect_image_paths(argv):
    if len(argv) > 1:
        return argv[1:]

    image_paths = []
    while True:
        index = len(image_paths) + 1
        path = f"calibration_images/chessboard{'0' if len(image_paths) < 9 else ''}{index}.jpg"
        if os.path.isfile(path):
            image_paths.append(path)
        else:
            break
    return image_paths


def main(argv):
    board_size = (6, 4)
    square_size = 30.0

    image_paths = collect_image_paths(argv)

    images = []
    print("\n正在加载标定图像...")
    for path in image_paths:
        image = cv2.imread(path)
        if image is not None:
            images.append(image)
            print(f"  已加载: {path}")

    if not images:
        print("\n错误：未能加载任何图像！", file=sys.stderr)
        print("请确保图像路径正确，或通过命令行参数提供图像文件。", file=sys.stderr)
        return -1

    print(f"\n共加载 {len(images)} 张图像")
    image_size = (images[0].shape[1], images[0].shape[0])

    print("\n开始检测棋盘角点...")
    print(f"棋盘尺寸: {board_size[0]} x {board_size[1]} 内角点")

    show_gui = is_gui_available()
    if not show_gui:
        print("\n未检测到可用图形会话，已自动禁用角点检测窗口显示（imshow）。")

    success_count, image_points = find_chessboard_corners(images, board_size, show_results=show_gui)

    if success_count < 3:
        print("\n错误：成功检测到角点的图像太少（至少需要3张）！", file=sys.stderr)
        print("请检查：", file=sys.stderr)
        print("  1. 棋盘尺寸设置是否正确", file=sys.stderr)
        print("  2. 图像质量是否足够好", file=sys.stderr)
        print("  3. 棋盘是否完整且清晰可见", file=sys.stderr)
        return -1

    print(f"\n成功检测: {success_count} / {len(images)} 张图像")

    object_points_pattern = generate_object_points(board_size, square_size)
    object_points = [object_points_pattern for _ in range(len(image_points))]

    print("\n正在执行相机标定...")
    rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = run_calibration(
        object_points,
        image_points,
        image_size,
    )

    print_calibration_results(camera_matrix, dist_coeffs, rms)

    output_file = "camera_calibration_result.xml"
    save_calibration_results(
        output_file,
        image_size,
        board_size,
        square_size,
        camera_matrix,
        dist_coeffs,
        rms,
    )

    # ==================== 计算相机外参（PnP） ====================

    ext_choice = input("\n是否要基于地面棋盘格计算相机外参？(y/n): ").strip()
    if ext_choice in ("y", "Y"):
        # 从专门的外参图像文件夹加载图像
        # 该文件夹独立于标定图像，专门存放用于外参计算的棋盘格照片
        ext_image_paths = []
        ext_idx = 1
        while True:
            path = f"extrinsic_images/chessboard{'0' if ext_idx < 10 else ''}{ext_idx}.jpg"
            if os.path.isfile(path):
                ext_image_paths.append(path)
                ext_idx += 1
            else:
                break

        if not ext_image_paths:
            print(
                "\n错误：extrinsic_images/ 文件夹中未找到外参图像！",
                file=sys.stderr,
            )
            print(
                "请将用于外参计算的棋盘格图像放入 extrinsic_images/ 文件夹，"
                "并按照 chessboard01.jpg, chessboard02.jpg ... 的命名规则命名。",
                file=sys.stderr,
            )
        else:
            print(f"\n从 extrinsic_images/ 加载了 {len(ext_image_paths)} 张外参图像")

            # 询问是否将外参保存到XML文件（提前询问，避免逐张询问）
            save_ext_choice = input("是否将外参保存到文件？(y/n): ").strip()
            save_ext = save_ext_choice in ("y", "Y")

            if save_ext:
                fs_ext = cv2.FileStorage(
                    "camera_extrinsic_result.xml", cv2.FILE_STORAGE_WRITE
                )

            # 遍历每张外参图像，依次检测角点并计算外参
            for i, ext_path in enumerate(ext_image_paths):
                # 加载图像
                ext_img = cv2.imread(ext_path)
                if ext_img is None:
                    print(f"  警告：无法加载 {ext_path}，跳过")
                    continue

                # 转换为灰度图并检测棋盘角点
                gray = cv2.cvtColor(ext_img, cv2.COLOR_BGR2GRAY)
                flags = (
                    cv2.CALIB_CB_ADAPTIVE_THRESH
                    | cv2.CALIB_CB_NORMALIZE_IMAGE
                    | cv2.CALIB_CB_FAST_CHECK
                )
                found, ext_corners = cv2.findChessboardCorners(gray, board_size, flags)

                if not found:
                    print(f"\n图像 {ext_path}: 未能检测到角点，跳过")
                    continue

                # 亚像素级角点优化
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.001
                )
                ext_corners = cv2.cornerSubPix(
                    gray, ext_corners, (11, 11), (-1, -1), criteria
                )

                # 使用PnP计算当前图像对应的相机外参
                rvec, tvec, R = compute_extrinsic_pnp(
                    object_points_pattern,  # 世界坐标（地面棋盘格角点）
                    ext_corners,            # 当前图像检测到的角点
                    camera_matrix,
                    dist_coeffs,
                )

                print(f"\n--- 外参图像: {ext_path} ---")
                print_extrinsic_results(rvec, tvec, R)

                # 保存外参到XML文件（以图像编号作为键名前缀）
                if save_ext:
                    prefix = f"extrinsic_{i + 1}"
                    cam_pos = -R.T @ tvec.reshape(3, 1)
                    fs_ext.write(f"{prefix}_image_path", ext_path)
                    fs_ext.write(f"{prefix}_rvec", rvec)
                    fs_ext.write(f"{prefix}_tvec", tvec)
                    fs_ext.write(f"{prefix}_rotation_matrix", R)
                    fs_ext.write(f"{prefix}_camera_position_in_world", cam_pos)

            if save_ext:
                fs_ext.release()
                print("\n外参已保存到：camera_extrinsic_result.xml")

    choice = input("\n是否要导出去畸变效果图？(y/n): ").strip()
    if choice in ("y", "Y"):
        save_undistortion_results(
            images,
            image_paths,
            camera_matrix,
            dist_coeffs,
            output_dir="undistortion_results",
        )

    print("\n标定完成！")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))