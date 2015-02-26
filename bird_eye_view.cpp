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

void distort(Mat &K,Mat &D,Mat &R,Mat &P,Size size,Mat &map1,Mat &map2)
{
	map1.create(size,CV_32FC1);
	map2.create(size,CV_32FC1);

    cv::Vec2d f, c;

    Matx33d camMat = K;
    f = Vec2d(camMat(0, 0), camMat(1, 1));
    c = Vec2d(camMat(0, 2), camMat(1, 2));

    Vec4d k = Vec4d::all(0);
	Matx33d dis_coff = D;
	k = Vec4d(dis_coff(0,0),dis_coff(0,1),dis_coff(0,2),dis_coff(0,3));

    cv::Matx33d RR  = cv::Matx33d::eye();

    R.convertTo(RR, CV_64F);

    cv::Matx33d PP = cv::Matx33d::eye();
   
  P.colRange(0, 3).convertTo(PP, CV_64F);

    cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);

    for( int i = 0; i < size.height; ++i)
    {
        float* m1f = map1.ptr<float>(i);
        float* m2f = map2.ptr<float>(i);
        short*  m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;

        double _x = i*iR(0, 1) + iR(0, 2),
               _y = i*iR(1, 1) + iR(1, 2),
               _w = i*iR(2, 1) + iR(2, 2);

        for( int j = 0; j < size.width; ++j)
        {
            double x = _x/_w, y = _y/_w;

            double r = sqrt(x*x + y*y);
            double theta = atan(r);

            double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
            double theta_d = theta * (1 + k[0]*theta2 + k[1]*theta4 + k[2]*theta6 + k[3]*theta8);

            double scale = (r == 0) ? 1.0 : theta_d / r;
            double u = f[0]*x*scale + c[0];
            double v = f[1]*y*scale + c[1];

            m1f[j] = (float)u;
            m2f[j] = (float)v;

            _x += iR(0, 0);
            _y += iR(1, 0);
            _w += iR(2, 0);
        }
    }
}

int main(int argc,char *argv[])
{
	return 0;
}





