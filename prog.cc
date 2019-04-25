#include <cstdlib>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include <armadillo>
using namespace std;
using namespace arma;

#define PI 3.14159265

cv::Mat getPerspectiveTransform(
    double pitch, double yaw, double roll,
    double fx, double fy, double kx, double ky,
    double wa, double ha, double wv, double hv, double scale
    ){
  // Camera Matrices
  mat K = {
    {fx, 0, kx},
    {0, fy, ky},
    {0, 0, 1}};
  mat K_inv = {
    {1/fx, 0, -kx/fx},
    {0, 1/fy, -ky/fy},
    {0, 0, 1}};
  // Rotation
  pitch = pitch / 180 * PI; double cp = cos(pitch); double sp = sin(pitch);
  yaw   = yaw   / 180 * PI; double cy = cos(yaw);   double sy = sin(yaw);
  roll  = roll  / 180 * PI; double cr = cos(roll);  double sr = sin(roll);
  mat R = {
    // ypr
    //{ cr*cy + sp*sr*sy,   cy*sr - cr*sp*sy, cp*sy},
    //{           -cp*sr,              cp*cr,    sp},
    //{ cy*sp*sr - cr*sy, - sr*sy - cr*cy*sp, cp*cy}
    // ryp
    {  cr*cy, cp*sr - cr*sp*sy, sp*sr + cp*cr*sy},
    { -cy*sr, cp*cr + sp*sr*sy, cr*sp - cp*sr*sy},
    {    -sy,           -cy*sp,            cp*cy}
  };
  // Pivot Point
  vec p0 = {wa/2, ha, 1};
  // Contrained Homography
  vec n = {0, -sp, cp};
  mat den = n.t() * (K_inv * p0);
  vec t_d = (R * (K_inv * p0) - K_inv * p0) / den(0,0);
  mat H = K * (R - t_d * n.t()) * K_inv;

  // Yielding mapping grid
  std::vector<cv::Point2f> src = {cv::Point2f(0, 0), cv::Point2f(wa, 0), cv::Point2f(0, ha), cv::Point2f(wa, ha)};
  std::vector<cv::Point2f> dst;
  for (const auto& pa : src) {
    vec pah = {pa.x, pa.y, 1.0};
    vec pvh = H * pah;
    dst.push_back(cv::Point2f((pvh(0)/pvh(2)-kx)*scale+wv/2, (pvh(1)/pvh(2) - ha)*scale+hv));
  }
  return cv::getPerspectiveTransform(src, dst);
}


int main(int argc, char** argv) {
#define IMAGE_FILE argv[1]
#define pitch atof(argv[2])
#define yaw   atof(argv[3])
#define roll  atof(argv[4])
#define fx    (2262.52)
#define fy    (2265.30)
#define cx    (1096.98)
#define cy    (513.137)
#define wa    (image.cols)
#define ha    (image.rows)
#define wv    (500)
#define hv    (2000)
#define scale ((float)(wv) / wa / 2.0)

  if (argc < 4) {
    printf("%s <image> <pitch> <yaw> <roll>", argv[0]);
    exit(0);
  }

  cv::Mat image = cv::imread(IMAGE_FILE);
  cv::Mat canvas;
  cv::Mat perspective = getPerspectiveTransform(
      pitch, yaw, roll,
      fx, fy, cx, cy,
      wa, ha, wv, hv, scale);
  cv::warpPerspective(image, canvas, perspective, cv::Size(wv, hv), cv::INTER_LINEAR);
  cv::imwrite("temp.png", canvas);
}
