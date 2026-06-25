#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp> // 用于相机标定函数
#include <opencv2/highgui.hpp> // 用于显示图像
#include <opencv2/imgproc.hpp> // 用于图像处理（如cvtColor）
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
                Size(-1, -1), // 没有死区
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
    
    time_t t; // 获取当前时间
    time(&t); // 获取当前时间戳
    struct tm* timeinfo = localtime(&t); // 转换为本地时间结构
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo); // 格式化时间字符串
    
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
        double scale = 800.0 / comparison.cols; // 调整宽度为800像素
        resize(comparison, display, Size(), scale, scale); // 等比例缩放
        
        // 添加文字说明
        putText(display, "Original", Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2); // 绿色字体
        putText(display, "Undistorted", Point(display.cols / 2 + 10, 30),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 0), 2);
        
        imshow("去畸变效果对比", display);
        
        int key = waitKey(0);
        if (key == 27) break; // ESC键退出
    }
    
    destroyWindow("去畸变效果对比");
}

/**
 * @brief 基于地面棋盘格，使用PnP算法计算相机的外参（旋转向量和平移向量）
 *
 * 地面棋盘格定义世界坐标系：棋盘平面为Z=0平面（地面），
 * X轴沿棋盘宽度方向，Y轴沿棋盘高度方向，Z轴垂直地面向上。
 *
 * @param objectPoints 棋盘角点的世界坐标（3D点集）
 * @param imagePoints  棋盘角点的图像坐标（2D点集）
 * @param cameraMatrix 相机内参矩阵（3x3）
 * @param distCoeffs   畸变系数
 * @param rvec 输出：旋转向量（Rodrigues形式，3x1）
 * @param tvec 输出：平移向量（3x1，单位与squareSize一致）
 * @return 是否成功计算
 */
bool computeExtrinsicPnP(
    const vector<Point3f>& objectPoints,
    const vector<Point2f>& imagePoints,
    const Mat& cameraMatrix,
    const Mat& distCoeffs,
    Mat& rvec,
    Mat& tvec)
{
    // 使用PnP算法求解相机位姿
    // SOLVEPNP_ITERATIVE: 基于Levenberg-Marquardt优化的迭代方法，精度较高
    cv::solvePnP(
        objectPoints,
        imagePoints,
        cameraMatrix,
        distCoeffs,
        rvec,
        tvec,
        false,                 // 不使用外部初始猜测值
        SOLVEPNP_ITERATIVE
    );

    return true;
}

/**
 * @brief 打印相机外参结果
 *
 * 显示旋转向量、旋转矩阵、平移向量，以及相机在世界坐标系中的位置。
 *
 * @param rvec 旋转向量（Rodrigues形式）
 * @param tvec 平移向量
 */
void printExtrinsicResults(const Mat& rvec, const Mat& tvec)
{
    // 将旋转向量转换为旋转矩阵（便于理解和使用）
    Mat R;
    Rodrigues(rvec, R);

    cout << "\n========== 相机外参（基于地面棋盘格PnP）==========" << endl;

    // 输出旋转向量（Rodrigues形式：旋转轴×旋转角）
    cout << "\n旋转向量 (Rodrigues形式):" << endl;
    cout << "  rx = " << rvec.at<double>(0, 0) << endl;
    cout << "  ry = " << rvec.at<double>(1, 0) << endl;
    cout << "  rz = " << rvec.at<double>(2, 0) << endl;

    // 输出旋转矩阵（世界坐标系 → 相机坐标系）
    cout << "\n旋转矩阵 R（世界坐标系 → 相机坐标系）:" << endl;
    cout << R << endl;

    // 输出平移向量（世界坐标系原点在相机坐标系中的坐标）
    cout << "\n平移向量 t（世界坐标系原点在相机坐标系中的位置，单位：mm）:" << endl;
    cout << "  tx = " << tvec.at<double>(0, 0) << " mm" << endl;
    cout << "  ty = " << tvec.at<double>(1, 0) << " mm" << endl;
    cout << "  tz = " << tvec.at<double>(2, 0) << " mm" << endl;

    // 计算相机光心在世界坐标系中的位置: C = -R^T * t
    // R是正交矩阵，其逆矩阵等于其转置
    Mat R_transpose = R.t();
    Mat cameraPos = -R_transpose * tvec;

    cout << "\n相机在世界坐标系中的位置（光心）:" << endl;
    cout << "  Xc = " << cameraPos.at<double>(0, 0) << " mm" << endl;
    cout << "  Yc = " << cameraPos.at<double>(1, 0) << " mm" << endl;
    cout << "  Zc = " << cameraPos.at<double>(2, 0) << " mm （相机距地面高度）" << endl;

    // 从旋转矩阵分解欧拉角（ZYX顺序：偏航Yaw → 俯仰Pitch → 翻滚Roll）
    double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0)
                   + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;

    double pitch, yaw, roll;
    if (!singular) {
        pitch = atan2(-R.at<double>(2, 0), sy);
        yaw   = atan2( R.at<double>(1, 0), R.at<double>(0, 0));
        roll  = atan2( R.at<double>(2, 1), R.at<double>(2, 2));
    } else {
        // 万向节死锁时的退化处理
        pitch = atan2(-R.at<double>(2, 0), sy);
        yaw   = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        roll  = 0;
    }

    // 弧度转角度
    pitch = pitch * 180.0 / CV_PI;
    yaw   = yaw   * 180.0 / CV_PI;
    roll  = roll  * 180.0 / CV_PI;

    cout << "\n相机欧拉角（ZYX顺序）:" << endl;
    cout << "  偏航角 Yaw   = " << yaw   << "°" << endl;
    cout << "  俯仰角 Pitch = " << pitch << "°" << endl;
    cout << "  翻滚角 Roll  = " << roll  << "°" << endl;

    cout << "\n===================================================" << endl;
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
    float squareSize = 30.0f; 
    
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
        // 从calibration_images目录读取所有的图像 chessboard01.jpg 一直到最后一个文件
        while(true) {
            string path = "calibration_images/chessboard" + string(imagePaths.size() < 9 ? "0" : "") + to_string(imagePaths.size() + 1) + ".jpg";
            if (ifstream(path).good()) {
                imagePaths.push_back(path);
            } else {
                break; // 没有更多图像了
            }
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

    // ==================== 计算相机外参（PnP） ====================

    cout << "\n是否要基于地面棋盘格计算相机外参？(y/n): ";
    char extChoice;
    cin >> extChoice;

    if (extChoice == 'y' || extChoice == 'Y') {
        // 从专门的外参图像文件夹加载图像
        // 该文件夹独立于标定图像，专门存放用于外参计算的棋盘格照片
        vector<string> extImagePaths;
        int extIdx = 1;
        while (true) {
            string path = "extrinsic_images/chessboard"
                + string(extIdx < 10 ? "0" : "")
                + to_string(extIdx) + ".jpg";
            if (ifstream(path).good()) {
                extImagePaths.push_back(path);
                extIdx++;
            } else {
                break;
            }
        }

        if (extImagePaths.empty()) {
            cerr << "\n错误：extrinsic_images/ 文件夹中未找到外参图像！" << endl;
            cerr << "请将用于外参计算的棋盘格图像放入 extrinsic_images/ 文件夹，" << endl;
            cerr << "并按照 chessboard01.jpg, chessboard02.jpg ... 的命名规则命名。" << endl;
        } else {
            cout << "\n从 extrinsic_images/ 加载了 "
                 << extImagePaths.size() << " 张外参图像" << endl;

            // 询问是否将外参保存到XML文件（提前询问，避免逐张询问）
            cout << "是否将外参保存到文件？(y/n): ";
            char saveExtChoice;
            cin >> saveExtChoice;
            bool saveExt = (saveExtChoice == 'y' || saveExtChoice == 'Y');

            FileStorage fs_ext;
            if (saveExt) {
                fs_ext.open("camera_extrinsic_result.xml", FileStorage::WRITE);
            }

            // 遍历每张外参图像，依次检测角点并计算外参
            for (size_t i = 0; i < extImagePaths.size(); i++) {
                // 加载图像
                Mat extImg = imread(extImagePaths[i]);
                if (extImg.empty()) {
                    cout << "  警告：无法加载 " << extImagePaths[i] << "，跳过" << endl;
                    continue;
                }

                // 转换为灰度图并检测棋盘角点
                Mat gray;
                cvtColor(extImg, gray, COLOR_BGR2GRAY);

                vector<Point2f> extCorners;
                bool found = cv::findChessboardCorners(
                    gray,
                    boardSize,
                    extCorners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK
                );

                if (!found) {
                    cout << "\n图像 " << extImagePaths[i]
                         << ": 未能检测到角点，跳过" << endl;
                    continue;
                }

                // 亚像素级角点优化
                cornerSubPix(
                    gray,
                    extCorners,
                    Size(11, 11),
                    Size(-1, -1),
                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001)
                );

                // 使用PnP计算当前图像对应的相机外参
                Mat rvec, tvec;
                computeExtrinsicPnP(
                    objectPointsPattern,  // 世界坐标（地面棋盘格角点）
                    extCorners,           // 当前图像检测到的角点
                    cameraMatrix,
                    distCoeffs,
                    rvec,
                    tvec
                );

                cout << "\n--- 外参图像: " << extImagePaths[i] << " ---";
                printExtrinsicResults(rvec, tvec);

                // 保存外参到XML文件（以图像编号作为节点名）
                if (saveExt) {
                    string nodeName = "extrinsic_" + to_string(i + 1);
                    fs_ext << nodeName << "{";
                    fs_ext << "image_path" << extImagePaths[i];
                    fs_ext << "rvec" << rvec;
                    fs_ext << "tvec" << tvec;
                    Mat R_ext;
                    Rodrigues(rvec, R_ext);
                    fs_ext << "rotation_matrix" << R_ext;
                    // 同时保存相机在世界坐标系中的位置 C = -R^T * t
                    Mat camPos = -R_ext.t() * tvec;
                    fs_ext << "camera_position_in_world" << camPos;
                    fs_ext << "}";
                }
            }

            if (saveExt) {
                fs_ext.release();
                cout << "\n外参已保存到: camera_extrinsic_result.xml" << endl;
            }
        }
    }

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
