# OpenCV 相机标定工具 (Camera Calibration Tool)

这是一个基于 OpenCV 和 C++ 实现的相机标定程序，使用张正友标定法（Zhang's method）。该工具可以计算相机的内参矩阵、畸变系数，并提供去畸变效果的测试功能。

## 📋 功能特点

*   **自动检测**：自动检测棋盘格角点，支持亚像素级精度优化。
*   **相机标定**：计算相机内参（焦距、主点）和畸变系数（径向和切向）。
*   **结果保存**：将标定结果保存为 XML 文件，方便后续读取和使用。
*   **效果验证**：提供去畸变（Undistort）的可视化对比功能，直观验证标定效果。
*   **详细文档**：包含详细的算法原理和流程说明（见 [相机标定流程详解.md](./相机标定流程详解.md)）。

## 🛠️ 依赖项

*   **C++ 编译器** (支持 C++14)
*   **CMake** (>= 3.10)
*   **OpenCV** (推荐 3.x 或 4.x 版本)

## 🚀 编译与运行

### 1. 编译代码

```bash
mkdir build
cd build
cmake ..
make
```

### 2. 准备标定图片

在项目根目录下创建一个名为 `calibration_images` 的文件夹（如果不存在），并将拍摄的棋盘格照片放入其中。
*   默认支持 `.jpg` 格式。
*   建议拍摄 10-20 张不同角度的图片。
*   **注意**：程序默认配置的棋盘格参数为 **内角点 6x4**，方格大小 **25mm**。如果您的棋盘格不同，请在运行前修改 `camera_calibration.cpp` 中的 `boardSize` 和 `squareSize` 变量并重新编译。

### 3. 运行程序

**方式一：自动读取默认目录**
直接运行程序，它会自动读取 `calibration_images/` 下的图片：
```bash
./camera_calibration
```

**方式二：手动指定图片路径**
```bash
./camera_calibration image1.jpg image2.jpg /path/to/image3.jpg
```

## 📄 输出结果

标定完成后，程序会在当前目录生成 `camera_calibration_result.xml` 文件，包含以下信息：
*   **Calibration Time**: 标定时间
*   **Camera Matrix**: 相机内参矩阵 ($3 \times 3$)
*   **Distortion Coefficients**: 畸变系数 ($k_1, k_2, p_1, p_2, k_3$)
*   **Reprojection Error**: 重投影误差 (RMS)

XML 文件示例：
```xml
<camera_matrix type_id="opencv-matrix">
  <rows>3</rows>
  <cols>3</cols>
  <data>
    1000.0 0.0 320.0
    0.0 1000.0 240.0
    0.0 0.0 1.0
  </data>
</camera_matrix>
```

## 📚 详细文档

关于相机标定的数学原理、程序流程图以及代码的详细解读，请参阅本地文档：
[相机标定流程详解.md](./相机标定流程详解.md)

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！
