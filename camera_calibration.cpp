

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

/**
 * @brief 从图像中检测棋盘角点
 * @param images 输入的图像列表
 * @param boardSize 棋盘内角点的尺寸（宽度和高度）
 * @param imagePoints 输出：所有图像中检测到的角点（图像坐标系）
 * @param showResults 是否显示检测结果
 * @return 成功检测到角点的图像数量
 */
int findChessboardCorners(
    const vector<Mat>& images,
    Size boardSize,
    vector<vector<Point2f>>& imagePoints,
    bool showResults = true)
{
    int successCount = 0;
    
    for (size_t i = 0; i < images.size(); i++) {
        Mat gray;
        // 转换为灰度图
        cvtColor(images[i], gray, COLOR_BGR2GRAY);
        
        vector<Point2f> corners;
        // 查找棋盘角点
        // CALIB_CB_ADAPTIVE_THRESH: 自适应阈值
        // CALIB_CB_NORMALIZE_IMAGE: 归一化图像
        // CALIB_CB_FAST_CHECK: 快速检查以避免不必要的计算
        bool found = cv::findChessboardCorners(
            gray,
            boardSize,
            corners,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK
        );
        
        if (found) {
            // 亚像素级角点优化
            // 使用5x5的搜索窗口，3x3的死区（避免奇异点）
            // 迭代终止条件：最多30次迭代或精度达到0.001
            cornerSubPix(
                gray,
                corners,
                Size(5, 5),
                Size(-1, -1),
                TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001)
            );
            
            imagePoints.push_back(corners);
            successCount++;
            
            cout << "图像 " << i + 1 << ": 成功检测到角点" << endl;
            
            // 可视化检测结果
            if (showResults) {
                Mat imageWithCorners = images[i].clone();
                drawChessboardCorners(imageWithCorners, boardSize, corners, found);
                
                // 调整显示窗口大小
                Mat display;
                resize(imageWithCorners, display, Size(), 0.5, 0.5);
                imshow("检测到的角点", display);
                waitKey(500); // 显示500ms
            }
        } else {
            cout << "图像 " << i + 1 << ": 未能检测到角点" << endl;
        }
    }
    
    if (showResults) {
        destroyWindow("检测到的角点");
    }
    
    return successCount;
}

/**
 * @brief 生成棋盘的世界坐标系（3D坐标）
 * @param boardSize 棋盘内角点的尺寸
 * @param squareSize 每个方格的实际大小（单位：毫米或其他单位）
 * @param objectPoints 输出：棋盘角点的3D坐标
 */
void generateObjectPoints(
    Size boardSize,
    float squareSize,
    vector<Point3f>& objectPoints)
{
    objectPoints.clear();
    
    // 生成棋盘角点的3D坐标
    // 假设棋盘位于z=0平面上
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            objectPoints.push_back(
                Point3f(j * squareSize, i * squareSize, 0)
            );
        }
    }
}

/**
 * @brief 执行相机标定
 * @param objectPoints 所有图像的世界坐标点
 * @param imagePoints 所有图像的图像坐标点
 * @param imageSize 图像尺寸
 * @param cameraMatrix 输出：相机内参矩阵
 * @param distCoeffs 输出：畸变系数
 * @param rvecs 输出：每幅图像的旋转向量
 * @param tvecs 输出：每幅图像的平移向量
 * @return 重投影误差（RMS）
 */
double calibrateCamera(
    const vector<vector<Point3f>>& objectPoints,
    const vector<vector<Point2f>>& imagePoints,
    Size imageSize,
    Mat& cameraMatrix,
    Mat& distCoeffs,
    vector<Mat>& rvecs,
    vector<Mat>& tvecs)
{
    // 执行相机标定
    // 返回值是重投影误差RMS（Root Mean Square）
    double rms = cv::calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs,
        0  // 标定标志位，0表示使用默认设置
    );
    
    return rms;
}

/**
 * @brief 保存标定结果到XML文件
 */
void saveCalibrationResults(
    const string& filename,
    Size imageSize,
    Size boardSize,
    float squareSize,
    const Mat& cameraMatrix,
    const Mat& distCoeffs,
    double rms)
{
    FileStorage fs(filename, FileStorage::WRITE);
    
    time_t t;
    time(&t);
    struct tm* timeinfo = localtime(&t);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
    
    fs << "calibration_time" << buffer;
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "rms_reprojection_error" << rms;
    
    fs.release();
    
    cout << "\n标定结果已保存到: " << filename << endl;
}

/**
 * @brief 显示标定结果
 */
void printCalibrationResults(
    const Mat& cameraMatrix,
    const Mat& distCoeffs,
    double rms)
{
    cout << "\n========== 标定结果 ==========" << endl;
    cout << "\n相机内参矩阵 (Camera Matrix):" << endl;
    cout << cameraMatrix << endl;
    
    cout << "\n畸变系数 (Distortion Coefficients):" << endl;
    cout << "k1: " << distCoeffs.at<double>(0) << endl;
    cout << "k2: " << distCoeffs.at<double>(1) << endl;
    cout << "p1: " << distCoeffs.at<double>(2) << endl;
    cout << "p2: " << distCoeffs.at<double>(3) << endl;
    cout << "k3: " << distCoeffs.at<double>(4) << endl;
    
    cout << "\n重投影误差 RMS (Reprojection Error): " << rms << " 像素" << endl;
    cout << "==============================\n" << endl;
}

/**
 * @brief 测试去畸变效果
 */
void testUndistortion(
    const vector<Mat>& images,
    const Mat& cameraMatrix,
    const Mat& distCoeffs)
{
    if (images.empty()) return;
    
    cout << "\n按任意键查看下一张图像的去畸变效果，按ESC退出..." << endl;
    
    for (size_t i = 0; i < images.size(); i++) {
        Mat undistorted;
        // 对图像进行去畸变处理
        undistort(images[i], undistorted, cameraMatrix, distCoeffs);
        
        // 拼接原图和去畸变后的图像进行对比
        Mat comparison;
        hconcat(images[i], undistorted, comparison);
        
        // 调整显示大小
        Mat display;
        double scale = 800.0 / comparison.cols;
        resize(comparison, display, Size(), scale, scale);
        
        // 添加文字说明
        putText(display, "Original", Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
        putText(display, "Undistorted", Point(display.cols / 2 + 10, 30),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
        
        imshow("去畸变效果对比", display);
        
        int key = waitKey(0);
        if (key == 27) break; // ESC键退出
    }
    
    destroyWindow("去畸变效果对比");
}

/**
 * @brief 主函数
 */
int main(int argc, char** argv)
{
    // ==================== 参数配置 ====================
    
    // 棋盘参数：内角点数量（宽 x 高）
    // 注意：这里是内角点数量，不是方格数量
    // 例如：9x6的棋盘有10x7=70个方格，但只有9x6=54个内角点
    Size boardSize(6, 4);
    
    // 棋盘方格的实际大小（单位：毫米）
    float squareSize = 25.0f; // 例如：25mm
    
    // 标定图像路径（可以根据需要修改）
    vector<string> imagePaths;
    
    // 方式1：从命令行参数读取图像路径
    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            imagePaths.push_back(argv[i]);
        }
    }
    // 方式2：手动指定图像路径（示例）
    else {
        cout << "用法示例：" << endl;
        cout << "  " << argv[0] << " image1.jpg image2.jpg image3.jpg ..." << endl;
        cout << "\n未提供图像，将尝试从默认路径读取..." << endl;
        
        // 从calibration_images目录读取所有jpg图像
        for (int i = 1; i <= 20; i++) {
            string path = "calibration_images/image_" + to_string(i) + ".jpg";
            imagePaths.push_back(path);
        }
    }
    
    // ==================== 加载图像 ====================
    
    vector<Mat> images;
    cout << "\n正在加载标定图像..." << endl;
    
    for (const string& path : imagePaths) {
        Mat img = imread(path);
        if (!img.empty()) {
            images.push_back(img);
            cout << "  已加载: " << path << endl;
        }
    }
    
    if (images.empty()) {
        cerr << "\n错误：未能加载任何图像！" << endl;
        cerr << "请确保图像路径正确，或通过命令行参数提供图像文件。" << endl;
        return -1;
    }
    
    cout << "\n共加载 " << images.size() << " 张图像" << endl;
    Size imageSize = images[0].size();
    
    // ==================== 检测棋盘角点 ====================
    
    cout << "\n开始检测棋盘角点..." << endl;
    cout << "棋盘尺寸: " << boardSize.width << " x " << boardSize.height << " 内角点" << endl;
    
    vector<vector<Point2f>> imagePoints;
    int successCount = findChessboardCorners(images, boardSize, imagePoints, true);
    
    if (successCount < 3) {
        cerr << "\n错误：成功检测到角点的图像太少（至少需要3张）！" << endl;
        cerr << "请检查：" << endl;
        cerr << "  1. 棋盘尺寸设置是否正确" << endl;
        cerr << "  2. 图像质量是否足够好" << endl;
        cerr << "  3. 棋盘是否完整且清晰可见" << endl;
        return -1;
    }
    
    cout << "\n成功检测: " << successCount << " / " << images.size() << " 张图像" << endl;
    
    // ==================== 生成世界坐标点 ====================
    
    vector<Point3f> objectPointsPattern;
    generateObjectPoints(boardSize, squareSize, objectPointsPattern);
    
    // 为每张成功检测的图像复制相同的世界坐标点
    vector<vector<Point3f>> objectPoints(imagePoints.size(), objectPointsPattern);
    
    // ==================== 执行相机标定 ====================
    
    cout << "\n正在执行相机标定..." << endl;
    
    Mat cameraMatrix;    // 相机内参矩阵 3x3
    Mat distCoeffs;      // 畸变系数 (k1, k2, p1, p2, k3)
    vector<Mat> rvecs;   // 每幅图像的旋转向量
    vector<Mat> tvecs;   // 每幅图像的平移向量
    
    double rms = calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs
    );
    
    // ==================== 显示和保存结果 ====================
    
    printCalibrationResults(cameraMatrix, distCoeffs, rms);
    
    // 保存标定结果
    string outputFile = "camera_calibration_result.xml";
    saveCalibrationResults(
        outputFile,
        imageSize,
        boardSize,
        squareSize,
        cameraMatrix,
        distCoeffs,
        rms
    );
    
    // ==================== 测试去畸变效果 ====================
    
    cout << "\n是否要查看去畸变效果？(y/n): ";
    char choice;
    cin >> choice;
    
    if (choice == 'y' || choice == 'Y') {
        testUndistortion(images, cameraMatrix, distCoeffs);
    }
    
    cout << "\n标定完成！" << endl;
    
    return 0;
}
