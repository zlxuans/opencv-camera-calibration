# 相机标定程序

使用棋盘标定板进行相机标定的C++程序。

## 功能特性

- 自动检测棋盘角点
- 亚像素级角点精度优化
- 计算相机内参矩阵和畸变系数
- 可视化角点检测结果
- 显示去畸变效果对比
- 保存标定结果到XML文件

## 依赖项

- OpenCV 3.0 或更高版本
- CMake 3.10 或更高版本
- C++14 编译器

## 安装OpenCV（Ubuntu）

```bash
sudo apt-get update
sudo apt-get install libopencv-dev
```

## 编译

```bash
mkdir build
cd build
cmake ..
make
```

## 准备标定图像

1. **打印棋盘标定板**：
   - 使用标准的黑白棋盘图案
   - 确保打印质量高，方格边界清晰
   - 建议将棋盘粘贴在平整的硬板上

2. **拍摄标定图像**：
   - 至少拍摄10-20张不同角度和位置的棋盘图像
   - 确保棋盘完整出现在图像中
   - 覆盖图像的不同区域（中心、边缘、角落）
   - 改变棋盘的倾斜角度和距离
   - 避免模糊和过曝

3. **组织图像文件**：
   ```bash
   mkdir calibration_images
   # 将拍摄的图像放入此目录
   ```

## 使用方法

### 方法1：通过命令行参数指定图像

```bash
./camera_calibration image1.jpg image2.jpg image3.jpg ...
```

### 方法2：从默认目录读取

将标定图像放在 `calibration_images/` 目录中，命名为 `image_1.jpg`, `image_2.jpg`, ...

```bash
./camera_calibration
```

## 配置参数

在 `camera_calibration.cpp` 的主函数中修改以下参数：

```cpp
// 棋盘内角点数量（宽 x 高）
Size boardSize(9, 6);  // 默认：9x6

// 棋盘方格的实际大小（单位：毫米）
float squareSize = 25.0f;  // 默认：25mm
```

**注意**：`boardSize` 是内角点数量，不是方格数量。例如，如果棋盘有10x7个方格，那么内角点数量是9x6。

## 输出结果

程序会生成 `camera_calibration_result.xml` 文件，包含以下信息：

- **camera_matrix**：相机内参矩阵（3x3）
  ```
  [fx  0  cx]
  [0  fy  cy]
  [0   0   1]
  ```
  - fx, fy：焦距（像素单位）
  - cx, cy：主点坐标

- **distortion_coefficients**：畸变系数
  - k1, k2, k3：径向畸变系数
  - p1, p2：切向畸变系数

- **rms_reprojection_error**：重投影误差（像素）
  - 通常小于1像素表示标定质量良好

## 标定结果评估

重投影误差（RMS）参考：
- < 0.5 像素：优秀
- 0.5 - 1.0 像素：良好
- 1.0 - 2.0 像素：可接受
- > 2.0 像素：需要改进（检查图像质量或重新拍摄）

## 示例

```bash
# 编译
mkdir build && cd build
cmake ..
make

# 运行标定
cd ..
./build/camera_calibration calibration_images/*.jpg

# 查看结果
cat camera_calibration_result.xml
```

## 程序流程

1. 加载标定图像
2. 在每张图像中检测棋盘角点
3. 进行亚像素级角点优化
4. 使用检测到的角点计算相机参数
5. 显示标定结果
6. 保存结果到XML文件
7. （可选）显示去畸变效果对比

## 常见问题

**Q: 无法检测到角点？**
- 检查棋盘尺寸参数是否正确
- 确保图像清晰，棋盘完整可见
- 尝试改善光照条件

**Q: 重投影误差过大？**
- 增加标定图像数量
- 确保图像覆盖不同角度和位置
- 检查棋盘是否平整
- 提高图像质量

**Q: 如何在自己的程序中使用标定结果？**
```cpp
// 读取标定结果
FileStorage fs("camera_calibration_result.xml", FileStorage::READ);
Mat cameraMatrix, distCoeffs;
fs["camera_matrix"] >> cameraMatrix;
fs["distortion_coefficients"] >> distCoeffs;
fs.release();

// 使用标定结果进行去畸变
Mat undistorted;
undistort(image, undistorted, cameraMatrix, distCoeffs);
```

## 参考资料

- [OpenCV相机标定教程](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)
- [张正友标定法论文](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/)

## 许可证

MIT License
