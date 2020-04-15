#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

#include <vector>
#include <string>
#include <dirent.h>		  // Used Nuget: Install-Package dirent
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/opencv.hpp>

typedef std::vector<cv::Point3f>  obj_points; // object points (3D) for one image
typedef std::vector<cv::Point2f>  img_points; // image point (2D) for one image

#define CHESS_ROWS 6
#define CHESS_COLS 9
#define BACKSPACE ' '

void			  find_corners(std::vector<obj_points>&, std::vector<img_points>&, cv::Size &img_size);
void			  get_corners(std::string img_name, std::vector<obj_points> &arr_obj_points, std::vector<img_points>&, cv::Size &img_size);
std::string	get_path(std::string message);
void			  calibrate_camera(cv::Mat &, cv::Mat &, std::vector<obj_points>&, std::vector<img_points>&, cv::Size &);
void 			  get_chess_size();

int main()
{
	std::vector<obj_points> arr_obj_points; //array of object points for an array of images
	std::vector<img_points> arr_img_points; //array of images points for an array of iamges
	cv::Mat cam_matrix = cv::Mat(3, 3, CV_32FC1);
	cv::Mat dist_coeffs; // This is what we need to undistort an image
	cv::Size img_size;

	// Get chess board size
	get_chess_size();
	// Go to image folder to get images for calibration
	find_corners(arr_obj_points, arr_img_points, img_size);

	// Start calibration
	calibrate_camera(cam_matrix, dist_coeffs, arr_obj_points, arr_img_points, img_size);

	// Save the cam_matrix and distort_coefficient
	cv::FileStorage file("calibration_matrix.xml", cv::FileStorage::WRITE);
	file << "camera_matrix" << cam_matrix << "distortion_coefficients" << dist_coeffs;
	file.release();

	// Save matrix
	cv::waitKey(100000);

	return 0;
}

void find_corners(std::vector<obj_points> &arr_obj_points, std::vector<img_points> &arr_img_points, cv::Size &img_size) {
	std::string path = get_path("Please enter image folder (leave blank to quit): ");
	// DIR *folder = opendir(path.c_str());
  DIR *folder = opendir("/home/tantrizio/workspace/CameraCalibration/realSenseImgs/");
	dirent *pent = NULL;
	if (folder != NULL) {
		// while there is still something to read in current dir
		while ( (pent = readdir(folder)) ) {
			if (pent){	// is readable 
				if (strstr(pent->d_name, ".jpg") != 0) { //and is image
					get_corners(path + pent->d_name, arr_obj_points, arr_img_points, img_size);
				}
			}
			else {
				printf("Error: Cannot read file correctly.\n");
				exit(3);
			}
		}
		closedir(folder);
	}
	else {
		printf("Error: could not open folder.\n");
	}
}

void get_corners(std::string img_name, std::vector<obj_points> &arr_obj_points, std::vector<img_points>& arr_img_points, cv::Size &img_size) {
	// Convert WINDOW PATH to UNIX PATH : C\\image.jpg  to ./image.jpg
	cv::Mat image = cv::imread(img_name, 1);
	if (!image.empty()) {
		// Size of the chess board
		cv::Size board_sz = cv::Size(CHESS_ROWS, CHESS_COLS);
		// Hold corner values returned from findChessCorners
		img_points corners;
		// Generate object points 
		obj_points corners_3d;
		for (int i = 0; i < CHESS_COLS*CHESS_ROWS; i++)
			corners_3d.push_back(cv::Point3f(i / CHESS_ROWS, i % CHESS_ROWS, 0.0f));

		// Find Chess corners
		bool found = cv::findChessboardCorners(image, board_sz, corners,
												cv::CALIB_CB_ADAPTIVE_THRESH |
												cv::CALIB_CB_FAST_CHECK |
												cv::CALIB_CB_NORMALIZE_IMAGE);
		// ***KEY*** 
		if (found)
		{
			cv::Mat gray;
			cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
			cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			std::cout << "Added image points for image " << img_name << " size " << image.size() << std::endl;
			arr_img_points.push_back(corners);
			arr_obj_points.push_back(corners_3d);
			// For Debug:
			cv::imshow("image", gray); // display on image window	
			cv::drawChessboardCorners(image, board_sz, corners, found);   // Draw corners on original image
			cv::imshow("calibrated", image);
			cv::waitKey(500);	// wait 2 secs
		}
		img_size = image.size();
	}
	else
		std::cout << "Could not read image " + img_name << std::endl;
}

void calibrate_camera(cv::Mat &cam_matrix, cv::Mat &dist_coeffs, std::vector<obj_points>& arr_obj_points, std::vector<img_points>& arr_img_points, cv::Size &img_size) {
	std::vector<cv::Mat> rotation_vectors;
	std::vector<cv::Mat> translation_vectors;
	cam_matrix.ptr<float>(0)[0] = 1;
	cam_matrix.ptr<float>(1)[1] = 1;
	cv::calibrateCamera(arr_obj_points, arr_img_points, img_size, cam_matrix, dist_coeffs, rotation_vectors, translation_vectors);
}

std::string get_path(std::string message) {
	std::string path;
	std::cout << message;
	path = "/home/tantrizio/workspace/CameraCalibration/realSenseImgs/";
	if (path.empty()) {
		std::cout << " Exiting program...\n" << std::endl;
		exit(101);
	}
	return path;
}

void	get_chess_size() {
	int rows, cols;
	bool good = false;
	do {
		std::cout << "Enter number of rows: ";
		std::cin >> rows;
		std::cout << "Enter number of columns: ";
		std::cin >> cols;
		if (rows > 0 && cols > 0) {
			good = true;
			printf("You have selected chess board size has %d rows and %d cols.\n", rows, cols);
#undef CHESS_ROWS
#define CHESS_ROWS rows
#undef CHESS_COLS
#define CHESS_COLS cols
		}
		else {
			printf("Please only enter positive numbers.\n");
		}
	} while (good == false);
}