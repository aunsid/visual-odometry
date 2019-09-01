#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <vector>
#include <glob.h> //number of png files
#include <string>
#include <filesystem>
#include <sstream> // sequential file opening

using namespace cv::xfeatures2d;
using namespace cv;
using namespace std;


double getAbsoluteScale(int frame_id)	{
  
  string line;
  int i = 0;
  ifstream myfile ("/home/aun/Downloads/data_odometry_gray/dataset/sequences/00/times.txt");
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}


void FindFeatureMatches(const Mat &image_l,
						const Mat & image_r,
						vector<KeyPoint> & p_l,
						vector<KeyPoint> & p_r,
						vector <DMatch> & good_matches){

	Mat descriptors_l, descriptors_r;
	Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(50);
	Ptr<DescriptorExtractor>descriptor = ORB::create();
	detector->detect(image_l, p_l);
	detector->detect(image_r, p_r);

	descriptor->compute(image_l, p_l, descriptors_l);
	descriptor->compute(image_r, p_r, descriptors_r);

	vector< vector<DMatch> > matches;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce"); 
	matcher->knnMatch ( descriptors_l, descriptors_r, matches,500 );
	Mat outimg1;

	good_matches.reserve(matches.size());  
	// cout<<matches.size()<<endl;
	double tresholdDist = 0.25 * sqrt(double(image_l.size().height*image_l.size().height + image_l.size().width*image_l.size().width));
	
	for (size_t i = 0; i < matches.size(); ++i)
	{ 
    	for (int j = 0; j < matches[i].size(); j++)
    	{
        	Point2f from = p_l[matches[i][j].queryIdx].pt;
        	Point2f to   = p_r[matches[i][j].trainIdx].pt;

        	//calculate local distance for each possible match
        	double dist = sqrt((from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y));

        	//save as best match if local distance is in specified area and on same height
        	if (dist < tresholdDist && abs(from.y-to.y)<5)
        	{
            	good_matches.push_back(matches[i][j]);
            	j = matches[i].size();
        	}
    	}
	}
	
    // drawMatches( image_l, p_l, image_r, p_r, good_matches, outimg1);
    // resizeWindow("ORB", 640,480);
    // imshow("ORB",outimg1);

    // waitKey(0);
}


void PoseEstimation2d(const vector<KeyPoint>& keypoints_1,
                      const vector<KeyPoint>& keypoints_2,
                      const vector<DMatch>  & matches,
                      const double & focal,
                      const Point2d & pp,
                      Mat &R, Mat &t){


	vector <Point2f> points1, points2;

	for ( int i= 0; i < (int)matches.size(); i++)
	{
		points1.push_back(keypoints_1[matches[i].queryIdx].pt);
		points2.push_back(keypoints_2[matches[i].trainIdx].pt);
	}

	Mat essential_matrix = findEssentialMat(points1, points2, focal, pp, RANSAC, 0.999, 1.0);
  	// cout << "essential_matrix is " << endl << essential_matrix << endl;
	recoverPose(essential_matrix, points1, points2, R, t,focal, pp);
}



int main(int argc, char **argv){
	/*
	Iterate over the images and call the necessary functions for vo


	*/
	double scale = 1.000;
	char text[100];
  	int fontFace = FONT_HERSHEY_PLAIN;
  	double fontScale = 1;
  	int thickness = 1;  
  	cv::Point textOrg(10, 60);

	// path for the images
	string path   = "/home/aun/Downloads/data_odometry_gray/dataset/sequences/00/image_0/";
	string all_png= "*.png";


	// use glob to get the number of files in the image
	glob_t gl;
	size_t number_images = 0;
	if(glob((path+all_png).c_str(), GLOB_NOSORT, NULL, &gl) == 0)
	  		number_images = gl.gl_pathc;
  	// cout<<number_images<<endl;

 	
  	double focal = 718.8560;
  	Point2d pp(607.1928, 185.2157);

  	Mat image_curr, image_next;
  	Mat  cum_R, cum_t;
  	Mat  R, t;
  	Mat traj = Mat::zeros(800, 800, CV_8UC3);

  	// iterate over the images and find the transformations, then draw the path.
	for (int image_number = 0; image_number < number_images-1; image_number++){
		
		
		// path of the next image
		stringstream path_image_curr, path_image_next;
		path_image_next<<path<<std::setfill('0')<<std::setw(6)<<image_number+1<<".png";
		
		// path of the current image
		if(image_number == 0){
			path_image_curr<<path<<std::setfill('0')<<std::setw(6)<<image_number<<".png";
	
			image_curr = imread(path_image_curr.str());
			image_next = imread(path_image_next.str());
		}
		else{

			image_curr = image_next.clone();
			image_next = cv::imread(path_image_next.str().c_str());
		}
		


		vector <KeyPoint> keypoints_1, keypoints_2;
		vector <DMatch> good_matches;
		FindFeatureMatches(image_curr, image_next, keypoints_1, keypoints_2, good_matches);
		cout<<"Number of matches: "<<good_matches.size()<<endl;


		
		
		PoseEstimation2d(keypoints_1, keypoints_2, good_matches,focal, pp,R, t);
  		cout<<"R:  "<<R<<endl;
  		cout<<"t:  "<<t<<endl;

  		scale = getAbsoluteScale(image_number);
  		scale = scale+1.0;
  		cout<<"scale"<<scale<<endl;

  		if (image_number == 0) {

      		cum_R = R.clone();
      		cum_t = t.clone();
    	}


   		else {

      		cum_t = cum_t + scale*(cum_R*t);
      		cum_R = R*cum_R;
      	}
    	
    	cout<<"if passed:"<<endl;
    	cout<<"R:  "<<cum_R<<endl;
  		cout<<"t:  "<<cum_t<<endl;
    	// t = t + scale*(R*t);
     //  	R = R*R;
  		int x = int(cum_t.at<double>(0)) + 400;
    	int y = int(cum_t.at<double>(2)) + 150;
    	// int z = int(cum_t.at<double>(1)) + 100;
    	circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    	rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0));
    	sprintf(text, "Coordinates:    x = %02fm       y = %02fm           z = %02fm", cum_t.at<double>(0), cum_t.at<double>(1),cum_t.at<double>(2));
    	// printf(text);
    	string temp_text(text);
    	cout<<"Image Number: "<<image_number<<endl;
    	cout<<"Text: "<<temp_text<<endl;
    	cv::putText(traj, temp_text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 2);

    	imshow( "Road facing camera",image_curr);
    	imshow( "Trajectory", traj );

    waitKey(1);


	}

  


	

	return 0;


	
}