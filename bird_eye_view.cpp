#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

double scale_factor;
double W,H;
double W0,H0;
double L;
void calc_virtual_cam()
{
	double f_x,f_y;
	double u_0,v_0;

//initiate virtual camera intrisinc
	f_x = L*W0/W;
	f_y = L*H0/H;
	u_0 = (W0-1)*0.5;
	v_0 = (H0-1)*0.5;
	Mat K(Size(3,3),CV_64FC1,Scalar::all(0));// intrisinc_matrix of vitual camera
	Mat K_inv;
	K.at<double>(0,0) = f_x;
	K.at<double>(1,1) = f_y;
	K.at<double>(0,2) = u_0;
	K.at<double>(1,2) = v_0;
	K.at<double>(2,2) = 1;
	K_inv = K.inv();
}

void distort()
{
}

int main(int argc,char *argv[])
{
	return 0;
}





