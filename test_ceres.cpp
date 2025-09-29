#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace std;
using namespace cv;

struct BallisticResidual {
    BallisticResidual(double t, double x_obs, double y_obs)
        : t_(t), x_obs_(x_obs), y_obs_(y_obs) {}

    template<typename T>
    bool operator()(const T* const params, T* residuals) const {
        T x0 = params[0];
        T y0 = params[1];
        T vx0 = params[2];
        T vy0 = params[3];
        T g   = params[4];
        T k   = params[5];

        T dt = T(t_);
        T e = ceres::exp(-k * dt);

        T x_pred = x0 + vx0 / k * (T(1.0) - e);
        T y_pred = y0 + (vy0 + g / k) / k * (T(1.0) - e) - g / k * dt;

        residuals[0] = x_pred - T(x_obs_);
        residuals[1] = y_pred - T(y_obs_);
        return true;
    }

private:
    const double t_, x_obs_, y_obs_;
};

int main() {
    const string video_path = "/home/shulin/TASK03/video.mp4";
    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        cerr << "Cannot open video: " << video_path << endl;
        return -1;
    }

    double fps = cap.get(CAP_PROP_FPS);
    int frame_w = (int)cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_h = (int)cap.get(CAP_PROP_FRAME_HEIGHT);

    Scalar lower_hsv(90, 150, 150);
    Scalar upper_hsv(110, 255, 255);

    Mat frame, hsv, mask;
    vector<Point2d> points_math;

    while (true) {
        bool ok = cap.read(frame);
        if (!ok || frame.empty()) break;

        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, lower_hsv, upper_hsv, mask);
        morphologyEx(mask, mask, MORPH_OPEN, Mat(), Point(-1,-1), 2);
        morphologyEx(mask, mask, MORPH_CLOSE, Mat(), Point(-1,-1), 1);

        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        Point2d center(-1, -1);
        double max_area = 0;
        for (auto &c : contours) {
            double a = contourArea(c);
            if (a > max_area) {
                Moments m = moments(c);
                if (m.m00 > 0) {
                    center = Point2d(m.m10 / m.m00, m.m01 / m.m00);
                    max_area = a;
                }
            }
        }

        if (center.x >= 0) {
            points_math.push_back(Point2d(center.x, frame_h - center.y)); 
        }
    }
    cap.release();

    cout << "Detected points: " << points_math.size() << endl;
    if (points_math.size() < 6) {
        cerr << "Too few points (<6). Exit.\n";
        return 0;
    }

    double x0_init = points_math.front().x;
    double y0_init = points_math.front().y;

    size_t useN = std::min((size_t)5, points_math.size() - 1);
    double sum_vx = 0, sum_vy = 0;
    for (size_t i = 0; i < useN; i++) {
        double dt = 1.0 / fps;
        sum_vx += (points_math[i + 1].x - points_math[i].x) / dt;
        sum_vy += (points_math[i + 1].y - points_math[i].y) / dt;
    }

    double vx0_init = sum_vx / useN;
    double vy0_init = sum_vy / useN;
    double g_init   = 500.0;
    double k_init   = 0.1;

    double params[6] = {x0_init, y0_init, vx0_init, vy0_init, g_init, k_init};

    ceres::Problem problem;
    ceres::LossFunction* loss = new ceres::HuberLoss(1.0);

    for (size_t i = 0; i < points_math.size(); i++) {
        double t = double(i) / fps;
        ceres::CostFunction* cf = new ceres::AutoDiffCostFunction<BallisticResidual, 2, 6>(
            new BallisticResidual(t, points_math[i].x, points_math[i].y));
        problem.AddResidualBlock(cf, loss, params);
    }

    problem.SetParameterLowerBound(params, 4, 100.0);
    problem.SetParameterUpperBound(params, 4, 1000.0);
    problem.SetParameterLowerBound(params, 5, 0.01);
    problem.SetParameterUpperBound(params, 5, 1.0);

    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    printf("Fitted parameters:\n");
    printf("x0 =  %.6f\n", params[0]);
    printf("y0 =  %.6f\n", params[1]);
    printf("vx0 = %.6f\n", params[2]);
    printf("vy0 = %.6f\n", params[3]);
    printf("g =   %.6f\n", params[4]);
    printf("k =   %.6f\n", params[5]);
    
    return 0;
}
