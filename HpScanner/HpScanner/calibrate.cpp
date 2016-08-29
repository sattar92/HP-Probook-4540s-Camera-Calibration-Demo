#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

int main()
{
	int numBoards = 15;
	int numCornersHor = 9;
	int numCornersVer = 6;

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);

	VideoCapture capture = VideoCapture(0);

	vector<vector<Point3f>> object_points;
	vector<vector<Point2f>> image_points;

	vector<Point2f> corners;
	int success = 0;

	Mat image, gray;

	capture >> image;

	vector<Point3f> obj;
	for (int j = 0; j<numSquares; j++)
		obj.push_back(Point3f(j / numCornersHor, j%numCornersHor, 0.0f));

	while (success < numBoards)
	{
		cvtColor(image,gray,CV_BGR2GRAY);

		bool found = findChessboardCorners(image,board_sz,corners,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if (found)
		{
			cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray, board_sz, corners, found);
		}

		imshow("win1", image);
		imshow("win2", gray);

		capture >> image;
		int key = waitKey(1);
		if (key == 27)

			return 0;

		if (found != 0)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);

			cout <<"Snap stored!" << endl;

			success++;

			if (success > numBoards)
				break;
		}

	}
	cout << "Start Calibration " << endl;

	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	vector<Mat> rvecs;
	vector<Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;

	double rms = calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

	
	cout << "RMS Error : " << rms << endl;
	FileStorage fs1("calib.yml", FileStorage::WRITE);
	fs1 << "CM1" << intrinsic;
	fs1 << "D1" << distCoeffs;
	fs1 << "R" << rvecs;
	fs1 << "T" << tvecs;

	cout << "Done Calibration" << endl;
	
	Mat imageUndistorted;
	while (1)
	{
		capture >> image;
		undistort(image, imageUndistorted, intrinsic, distCoeffs);

		imshow("win1", image);
		imshow("win2", imageUndistorted);
		waitKey(1);
	}

	cin.get();
	capture.release();
	return 0;
}