#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;


int main(){

	int numBoards = 15;
	int board_w = 9;
	int board_h = 6;
	Mat img, gray;
	
	int success = 0, k = 0;
	bool found = false;

	VideoCapture cap = VideoCapture(0);

	Size board_sz = Size(board_w, board_h);
	int board_n = board_w*board_h;

	vector<vector<Point3f> > object_points;
	vector<vector<Point2f> > imagePoints;
	vector<Point2f> corners;
	vector<Point3f> obj;
	
	for (int j = 0; j<board_n; j++)
	{
		obj.push_back(Point3f(j / board_w, j%board_w, 0.0f));
	}

	while (1)
	{
		cap >> img;
		cvtColor(img, gray, CV_BGR2GRAY);

		found = findChessboardCorners(gray, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		
		if(found)
		{
			cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray, board_sz, corners, found);
		}
		
		imshow("image1", gray);

		/*
		k = waitKey(10);
		if (found)
		{
			k = waitKey(10);
		}
		if (k == 27)
		{
			break;
		}
		*/
		if (found != 0)
		{
			imagePoints.push_back(corners);
			object_points.push_back(obj);
			printf("Corners stored\n");
			success++;

			if (success >= numBoards)
			{
				break;
			}
		}
	
		waitKey(500);
	
	}

	cout <<"Starting Calibration\n";
	Mat CM1 = Mat(3, 3, CV_64F);
	Mat D1;
	Mat R, T;

	//double totalAvgErr = computeReprojectionErrors(object_points, imagePoints, R, T, CM1, D1, reprojErrs);
	double rms = calibrateCamera(object_points, imagePoints, gray.size(), CM1,
		D1, R, T, s.flag | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	cout << "RMS Error" << rms << endl;
	FileStorage fs1("Hpcalib.yml", FileStorage::WRITE);
	fs1 << "CM1" << CM1;
	fs1 << "D1" << D1;
	fs1 << "R" << R;
	fs1 << "T" << T;
	cout << "Done Calibration\n";

	destroyAllWindows();
	cap.release();
	cin.get();
}