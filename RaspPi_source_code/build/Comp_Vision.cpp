#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <iostream>             // for cout
#include <atomic>
#include <thread>
#include <unordered_map>
#include <cmath>
#include <chrono>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <string.h>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <netdb.h>

#include <stdio.h>  /* defines FILENAME_MAX */
// #define WINDOWS  /* uncomment this line to use it for windows.*/ 
#ifdef _WIN32
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

#define CAM_WIDTH 424 //424
#define CAM_HEIGHT 240 //240
#define CAM_WIDTH_DMODE 480
#define CAM_HEIGHT_DMODE 270 
#define CAM_FPS 30

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace rs2;

std::string GetCurrentWorkingDir(void) {
	char buff[FILENAME_MAX];
	GetCurrentDir(buff, FILENAME_MAX);
	std::string current_working_dir(buff);
	return current_working_dir;
}

struct FPS
{
	chrono::time_point<chrono::steady_clock> start, end;
	chrono::duration<double> duration;
	double fps, fps_current;

	FPS()
	{
		start = chrono::steady_clock::now();
	}
	
	double get_current_fps(){
	    fps_current = 1 / ((chrono::steady_clock::now() - start).count());
	    return fps_current;
	}

	~FPS()
	{
		end = chrono::steady_clock::now();
		duration = end - start;
		double sec = duration.count();
		fps = 1 / sec;
		//cout << "FPS counter: " << fps << " FPS" << endl;
	}
};

bool is_Box_Area_Clear(Mat _roi);
vector<KeyPoint> blob_detection(Mat source_im);
int client, server;
bool vibrating = false;

void *startSocketComm(void *mSocket_info) {
    int bufsize = 1024;
    char buffer[bufsize];
    int count = 0;
    int int_count = 0;
    int portnum = 63123;
    struct sockaddr_in server_addr;
    socklen_t size;
	
    //init socekt

    client = socket(AF_INET, SOCK_STREAM, 0);

    if(client < 0){
        cout << "Error Establishing the connection" << endl;
    }

    cout << "server Socket connection created" << endl;

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr.sin_port = htons(portnum);


    //biding soket

    while(bind(client,(struct sockaddr*)&server_addr,sizeof(server_addr)) < 0){
        cout << "Error Biding Socket" << endl;
    }

    size= sizeof(server_addr);
    cout << "waiting for clients" << endl;

    //listening for sokets

    if(listen(client,1) == -1){
		cout << "Can't listen!" << endl;
	}
    
    //accept client

    server = accept(client, (struct sockaddr*)&server_addr, &size);

    if(server < 0){
        cout<< "Error Accepting the client" << endl;
    }
    
	while(server > 0)
	{
		if (count == 0){
			memset(buffer, 0, bufsize); // clearing the buffer
			strcpy(buffer,"server connected---\n");
			send(server,buffer,bufsize,0);
			cout << "connected with the client!" << endl;
		}
		count += 1;
		if (count % 30000 == 0){
			int_count += 1;
			//cout << "Counting..." << int_count << endl;
			/*memset(buffer, 0, bufsize); // clearing the buffer
			strcpy(buffer,"1\n");
			if(send(server,buffer,sizeof(buffer),0) < 0){
				close(client);
				cout << "send() to client failed!" << endl;
				cout << "Conection Terminated..." << endl;
				cout << "Goodbye..." << endl;
				break;
			}*/
			memset(buffer, 0, bufsize); // clearing the buffer
			recv(server, buffer, sizeof(buffer), 0);
			cout << "Acknowledgement received: " << buffer << endl;
			char end_str[] = "end";
			if(strstr(buffer, end_str) != NULL){
			    memset(buffer, 0, bufsize); // clearing the buffer
			    strcpy(buffer,"0\n");
			    send(server,buffer,bufsize,0);
				cout << "Socket Conection Terminated..." << endl;
				cout << "Goodbye..." << endl;
				close(server);
				server = accept(client, (struct sockaddr*)&server_addr, &size);
				count = 0;
				int_count = 0;
			}
		}
		if (count == 30000)
			count = 1;
		
	}
	
	pthread_exit(NULL);
	
}

int main() try {

    // Vibration related variables
    double min_vibration_delay = 200;
    double vibration_delay = 0;
    bool zebra_crossing = false;

    // Socket Communication Setup
    
    int portnum = 64123;
    int bufsize = 1024;
    char buffer[bufsize];
    
    /// calling thread for socket setup
    pthread_t thread;
    int rc;
    rc = pthread_create(&thread, NULL, startSocketComm, NULL);
    cout << "Calling thread..\n" << endl;
	if (rc) {
		cout << "Error:unable to create thread," << rc << endl;
		exit(-1);
	}
    
    /////////////////////////////

	// Declare filters
	rs2::colorizer color_map;
	color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2); // 2: White-to-Black
	color_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0);

	rs2::spatial_filter spat_filter;
	if (spat_filter.supports(RS2_OPTION_HOLES_FILL)) {
		rs2::option_range option_range = spat_filter.get_option_range(RS2_OPTION_HOLES_FILL);
		spat_filter.set_option(RS2_OPTION_HOLES_FILL, 2); // 5 = Fill al the zero pixels
		std::cout << "Spatial Filter OK" << endl;
	}

	// Define transformations from and to Disparity domain
	rs2::disparity_transform depth2disparity;
	rs2::disparity_transform disparity2depth(false);

	rs2::hole_filling_filter hole_filter;
	hole_filter.set_option(RS2_OPTION_HOLES_FILL, 2); // 2: Nearest from around

	rs2::decimation_filter dec_filter;
	if (dec_filter.supports(RS2_OPTION_FILTER_MAGNITUDE)) {
		rs2::option_range option_range = dec_filter.get_option_range(RS2_OPTION_FILTER_MAGNITUDE);
		dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);
		std::cout << "Decimation Filter OK" << endl;
	}

	rs2::temporal_filter temp_filter;

	rs2::context ctx;
	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
	if (list.size() == 0)
		throw std::runtime_error("No device detected. Is it plugged in?");
	rs2::device dev = list.front();

	//Contruct a pipeline which abstracts the device
	rs2::pipeline pipe;
	//Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;

	//Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, CAM_WIDTH_DMODE, CAM_HEIGHT_DMODE, RS2_FORMAT_Z16, CAM_FPS);
	cfg.enable_stream(RS2_STREAM_COLOR, CAM_WIDTH, CAM_HEIGHT, RS2_FORMAT_BGR8, CAM_FPS);

	//Instruct pipeline to start streaming with the requested configuration
	auto profile = pipe.start(cfg);

	// Set the device to High Accuracy preset
	auto dth_sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();

	rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
	rs2::frame current_depth_frame = data.get_depth_frame(); // Take the depth frame from the frameset
	rs2::frame current_rgb_frame = data.get_color_frame(); // Take the rgb frame from the frameset

	for (int i = 0; i < 20; i++) {
		data = pipe.wait_for_frames();
	}

	int counter = 0;
	double threshold = 80;
	double max_threshold = 100;

	vector<Point> min_loc(4), max_loc(4); // Max and Min location of pixels with highest score

	cout << GetCurrentWorkingDir() << endl;

	//Mat image;
	//Mat image2;
	Mat result, red_templ, red_templ_resized, green_templ, green_templ_resized, image;
	Mat lookUpTable(1, 256, CV_8U);
	Mat edge_im;
	Mat thresholded_block, blurred, dilated, eroded;
	Mat gray;
	Mat red_mask1, red_mask2, red_mask;
	Mat green_mask;
	Mat mask;
	Mat res;
	Mat drawing;

	/*********************************************** NON Blocking Processing ***********************************************/
	// After initial post-processing, frames will flow into this queue:
	rs2::frame_queue postprocessed_frames(10, true);
	rs2::frame_queue rgb_frames_pool(10, true);

	// Alive boolean will signal the worker threads to finish-up
	std::atomic_bool alive{ true };

	// processing thread will fetch frames from the camera,
	// apply post-processing and send the result to the main thread for rendering
	// It recieves synchronized (but not spatially aligned) pairs
	// and outputs synchronized and aligned pairs
	std::thread processing_thread([&]() {
		while (alive)
		{
			// Fetch frames from the pipeline and send them for processing
			rs2::frameset data = pipe.wait_for_frames();
			rs2::frame _rgb_frame = data.get_color_frame(); // Take the rgb frame from the frameset
			rs2::frame _depth_frame = data.get_depth_frame(); // Take the depth frame from the frameset
			if (!_depth_frame) { // Should not happen but if the pipeline is configured differently
				cout << "NO DEPTH FRAME FOUND......!!!" << endl;
				return;       //  it might not provide depth and we don't want to crash
			}
			if (!_rgb_frame) {
				cout << "NO RGB FRAME FOUND......!!!" << endl;
				return;    
			}
			// Decimation will reduce the resultion of the depth image,
			// closing small holes and speeding-up the algorithm
			rs2::frame _filtered_frame = dec_filter.process(_depth_frame);
			// Apply spatial filtering
			rs2::frame _filtered1_frame = spat_filter.process(_filtered_frame);
			// Apply temporal filtering
			rs2::frame _filtered2_frame = temp_filter.process(_filtered1_frame);
			// If we are in disparity domain, switch back to depth
			//data = data.apply_filter(disparity2depth);
			rs2::frame _filtered3_frame = hole_filter.process(_filtered2_frame);

			// Send resulting frames for visualization in the main thread
			postprocessed_frames.enqueue(_filtered3_frame);
			rgb_frames_pool.enqueue(_rgb_frame);

			//cout << "DEPTH FRAME FOUND!" << endl;
		}

		cout << "Terminating Processing Thread..." << endl;
	});
	
	//string path_im = GetCurrentWorkingDir() + "/data/traffic_lights/traffic_light13.jpg";
	string path_red_templ = GetCurrentWorkingDir() + "/data/traffic_lights/red_light_template2.jpg";
	string path_green_templ = GetCurrentWorkingDir() + "/data/traffic_lights/green_light_template4.jpg";
	//image = imread(path_im, IMREAD_COLOR);   // Read the file
	red_templ = imread(path_red_templ, IMREAD_COLOR);	// Read the red template
	green_templ = imread(path_green_templ, IMREAD_COLOR);	// Read the green template

	//cout << path_im << endl;
	//cout << path_templ << endl;

				
	uchar* p = lookUpTable.ptr();
	for (int i = 0; i < 256; ++i)
		p[i] = saturate_cast<uchar>(pow(i / 255.0, 2.2) * 255.0);
		//LUT(templ, lookUpTable, templ);

	if (!red_templ.data)	// Check for invalid input
	{
		cout << "Could not open or find the image for red template" << std::endl;
		return -1;
	}
	else if (!green_templ.data)	// Check for invalid input
	{
		cout << "Could not open or find the image for green template" << std::endl;
		return -1;
	}

	/*namedWindow("App", WINDOW_NORMAL);
	namedWindow("Downsampling", WINDOW_NORMAL);
	namedWindow("Image", WINDOW_NORMAL);
	namedWindow("Colormap", WINDOW_NORMAL);*/

	// Main Loop
	while (alive) // Application still alive?
	{
		FPS fps_counter;
		counter++;
		bool middle_path_clear = false;

		int k = waitKey(1);
		//cout << k << endl;
		if (k == 27) { // # wait for ESC key to exit and terminate program
			destroyAllWindows();
			alive = false;
			break;
		}

		try {

			// Fetch the latest available DEPTH post-processed frameset
			if (postprocessed_frames.poll_for_frame(&current_depth_frame)) {

				rs2::frame depth_frame = current_depth_frame;
				rs2::frame colored_depth = color_map.colorize(current_depth_frame);

				Mat dth_fltred_im(Size(CAM_WIDTH_DMODE, CAM_HEIGHT_DMODE), CV_8UC3, (void*)colored_depth.get_data(), Mat::AUTO_STEP);
				float depth_scale = dth_sensor.get_depth_scale();
				Mat distances(Size(CAM_WIDTH_DMODE, CAM_HEIGHT_DMODE), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
				distances.convertTo(distances, CV_64F);
				distances = distances * depth_scale;

				float center_depth = distances.at<double>(distances.rows / 2, distances.cols / 2);
				//cout << "center at " << fixed << setprecision(3) << center_depth << " m" << endl;
				circle(dth_fltred_im, Point(distances.cols / 2, distances.rows / 2), 2, Scalar(0, 0, 255));

				Mat downSmpled_depth_im;
				pyrDown(dth_fltred_im, downSmpled_depth_im, Size(dth_fltred_im.cols / 2, dth_fltred_im.rows / 2)); // Downsampling the depth image
				//flip(downSmpled_depth_im, downSmpled_depth_im, 1);

				auto roi_width = downSmpled_depth_im.cols / 3;
				auto roi_height = downSmpled_depth_im.rows;
				Rect left_side(0, 0, roi_width, roi_height);
				Rect middle_side(roi_width, 0, roi_width, roi_height);
				Rect right_side(2 * roi_width, 0, roi_width, roi_height);

				Mat left_im = downSmpled_depth_im(left_side);
				Mat middle_im = downSmpled_depth_im(middle_side);
				Mat right_im = downSmpled_depth_im(right_side);

				//namedWindow("left", WINDOW_NORMAL);
				//namedWindow("middle", WINDOW_NORMAL);
				//namedWindow("right", WINDOW_NORMAL);
				//imshow("left", left_im);
				//imshow("middle", middle_im);
				//imshow("right", right_im);

				middle_path_clear = is_Box_Area_Clear(middle_im);

				if (!middle_path_clear) {
					bool left_path_clear = is_Box_Area_Clear(left_im);

					if (!left_path_clear) {
						bool right_path_clear = is_Box_Area_Clear(right_im);

						if (!right_path_clear) {
							cout << "NOWHERE TO GO.. I'M STUCK :(" << endl;
							char msg[1024];
				            memset(msg, 0, 1024); // clearing the buffer
			                strcpy(msg,"6\n");
			                if (server > 0 and !vibrating and !zebra_crossing){
			                    send(server,msg,sizeof(msg),0);     // send "6" to Android App
			                    vibrating = true;
							}
						}else{
							cout << "GO RIGHT! ==>>" << endl;
							char msg[1024];
				            memset(msg, 0, 1024); // clearing the buffer
			                strcpy(msg,"5\n");
			                if (server > 0 and !vibrating and !zebra_crossing){
			                    send(server,msg,sizeof(msg),0);     // send "5" to Android App
			                    vibrating = true;
			                }
						}
					}else{
						cout << "GO LEFT! <<==" << endl;
						char msg[1024];
				        memset(msg, 0, 1024); // clearing the buffer
			            strcpy(msg,"4\n");
			            if (server > 0 and !vibrating and !zebra_crossing){
			                send(server,msg,sizeof(msg),0);     // send "4" to Android App
			                vibrating = true;
					    }
                    }
				}else{
					cout << "MIDDLE PATH CLEAR" << endl;
				    char msg[1024];
				    memset(msg, 0, 1024); // clearing the buffer
			        strcpy(msg,"7\n");
			        if (server > 0 and !vibrating and !zebra_crossing){
			            send(server,msg,sizeof(msg),0);     // send "7" to Android App
			            //vibrating = true;
					}
				}

				//imshow("App", dth_fltred_im);
				//imshow("Downsampling", downSmpled_depth_im);
			}

			//  Fetch the latest available RGB frameset
			if (rgb_frames_pool.poll_for_frame(&current_rgb_frame)) {

				// Creating OpenCV Matrix from a color image
				Mat rgb_im(Size(CAM_WIDTH, CAM_HEIGHT), CV_8UC3, (void*)current_rgb_frame.get_data(), Mat::AUTO_STEP);

				if (!rgb_im.data) {
					cout << "Could not open or find the frame" << std::endl;
					return -1;
				}

				/*	Block-Based Hough Transform Algorithm, based on the paper of:
					Wu, X., Hu, R., & Bao, Y. (2019).
					"Block-Based Hough Transform for Recognition of Zebra Crossing in Natural Scene Images."
				*/
				Rect roi(cvRound(CAM_WIDTH * 0.1), cvRound(CAM_HEIGHT * 0.5), cvRound(CAM_WIDTH * 0.8), cvRound(CAM_HEIGHT * 0.5));	// Definition of ROI ~ 50% height, 80% width
				Mat roiMat = rgb_im(roi);

				//Mat roiMat = image2(roi);
				cvtColor(roiMat, roiMat, COLOR_BGR2GRAY);

				// Number of blocks
				int n_divisions = 3; // it defines the size of the block used
				// Definition of pixel_score matrix;
				Mat total_pixel_score = roiMat * 0;
				/*total_pixel_score.create(CAM_HEIGHT * 0.5, CAM_WIDTH, CV_32SC1);*/

				bool zebra_found = false;

				for (auto j = 0; j <= 0.8 * CAM_WIDTH - cvRound(0.8 * CAM_WIDTH / n_divisions); j = j + 75) {
					// Definition of Block
					Rect block(j, 0, cvRound(0.8 * CAM_WIDTH / n_divisions), cvRound(CAM_HEIGHT * 0.5));
					Mat blockMat = roiMat(block);
					///Mat blockMat = roiMat;

					// Adaptive Thresholding for Binaryzation
					adaptiveThreshold(blockMat, thresholded_block, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 7, 7);
					// Gaussian Filtering for Noise Reduction
					///GaussianBlur(thresholded_block, blurred, Size(3, 3), 0, 0);
					// Apply Morphological Transformation (Dilation and Erosion)
					dilate(thresholded_block, dilated, getStructuringElement(MORPH_RECT, Size(3, 3)));
					erode(dilated, eroded, getStructuringElement(MORPH_RECT, Size(3, 3)));
					// Edge detection with Canny Filter
					Canny(eroded, edge_im, 50, 300, 3);

					// Release Mat files
					blockMat.release();

					// Parallel Lines Detection
					vector<Point3f> lines; // < rho, theta, votes>

					double min_theta = 60;	// min limit of angle theta in degrees
					double max_theta = 120;	// max limit of angle theta in degrees

					HoughLines(edge_im, lines, 1, CV_PI / 180, threshold, 0, 0, min_theta * CV_PI / 180, max_theta * CV_PI / 180);

					sort(lines.begin(), lines.end(),					// sort "lines" depending on 'votes' in descending order
						[](const cv::Point3f &a, const cv::Point3f &b)
					{
						return (a.z > b.z);
					});

					vector<Point3f> lines_all = lines;

					// Declaring iterator to a vector 
					vector<Point3f>::iterator ptr = lines.begin();
					if (lines.size() < 10) {
						//std::cout << "Too few lines FOUND" << endl;
						threshold -= 10;		// Lower the threshold in order to increase the chance of finding lines
						continue;
					}
					else {
						advance(ptr, 9);
						lines.erase(ptr, lines.end() - 1);		// Keep only the first 10 lines with greater 'votes'
						if (threshold < max_threshold)
							threshold += 5;		// Increase the threshold in order to find only stronger lines
					}

					std::list<float> theta_list;
					for (auto i = 0; i < lines.size(); i++) {
						theta_list.push_back(lines[i].y);
					}
					theta_list.sort();

					unordered_map<float, size_t> counts;
					for (auto v : theta_list)
						++counts[v];
					theta_list.unique();		// Remove duplicate values

					int dom_value = 0;
					float dom_theta;
					for (auto th : theta_list) {
						if (dom_value < counts.at(th)) {
							dom_value = counts.at(th);
							dom_theta = th;
						}
					}

					///vector<Point3f> paral_lines;
					vector<int> votes;
					votes.clear();
					int c = 0;
					for (auto i = 0; i < lines_all.size(); i++) {
						if (lines_all[i].y == dom_theta) {
							///paral_lines.emplace_back(lines_all[i]);
							votes.emplace_back(lines_all[i].z);
							c++;
						}
					}

					sort(votes.begin(), votes.end(),					// sort "votes" depending on number of 'votes' in descending order
						[](const int &a, const int &b)
					{
						return (a > b);
					});


					if (votes.size() > 2 && votes[2] > 60) {
						zebra_found = true;
						zebra_crossing = true;
						cout << "There are parallel lines at angle " << dom_theta * (180.0 / CV_PI) << endl;
						total_pixel_score(block) = 0.2 * total_pixel_score(block) + votes[2] * 0.8;		// fomula for pixel-wise score calculation
					}else{
					    zebra_crossing = false;
						continue;
					}
				}

				if (zebra_found) {
					double min, max;
					minMaxLoc(total_pixel_score.row(0), &min, &max, &min_loc[0], &max_loc[0]);

					max_loc[1] = max_loc[0];
					max_loc[2] = max_loc[1];
					max_loc[3] = max_loc[2];

					Point max_loc_mean = max_loc[0] + max_loc[1] + max_loc[2] + max_loc[3];
					max_loc_mean.x = max_loc_mean.x / 4;
					max_loc_mean.y = max_loc_mean.y / 4;

					compare(total_pixel_score, max, total_pixel_score, CMP_EQ);
					int non_zero = countNonZero(total_pixel_score.row(0));

					Point pt1 = max_loc_mean + Point(cvRound(CAM_WIDTH * 0.2) / 2 + non_zero / 2, CAM_HEIGHT / 2);
					Point pt2 = max_loc_mean + Point(cvRound(CAM_WIDTH * 0.2) / 2 + non_zero / 2, CAM_HEIGHT / 2 + total_pixel_score.rows - 1);

					line(rgb_im, pt1, pt2, Scalar(155, 155, 155), 2, LINE_AA);
					//line(image2, pt1, pt2, Scalar(155, 155, 155), 2, LINE_AA);

				}

				/************************************ Traffic Light Detection ************************************/
				
				if(zebra_found){

				    Rect upper_roi(cvRound(CAM_WIDTH * 0.1), 0, cvRound(CAM_WIDTH * 0.8), cvRound(CAM_HEIGHT * 0.4));	// Definition of ROI ~ upper 40% height, 80% width
				    Mat upper_roi_mat = rgb_im(upper_roi);		// The detector will scan this ROI for traffic light
				    //resize(image, image, Size(CAM_WIDTH, CAM_HEIGHT));
				    //Mat upper_roi_mat = image(upper_roi);		// The detector will scan this ROI for traffic light

				    //Converting image from BGR to GRAY color space.
				    cvtColor(upper_roi_mat, gray, COLOR_BGR2GRAY);

				    //// Noise Removal // Paper "Detection of Traffic Lights for Vision-Based Car Navigation System" --> https://link.springer.com/content/pdf/10.1007%2F11949534_68.pdf 
				    
				    // Apply TOP-HAT Morphological Operation: Paper "Traffic light recognition using image processing compared to learning processes" --> https://ieeexplore.ieee.org/abstract/document/5353941
				    int morph_elem = MORPH_RECT; // Element --> 0: Rect, 1: Cross, 2: Ellipse 
				    int morph_size = 3;
				    int morph_operator = MORPH_TOPHAT; // Operator --> 2: Opening, 3: Closing, 4: Gradient, 5: Top Hat, 6: Black Hat
				    Mat element = getStructuringElement(morph_elem, Size(2 * morph_size + 1, 2 * morph_size + 1));
				    Mat post_gray;
				    morphologyEx(gray, post_gray, morph_operator, element);

				    //namedWindow("Morphological Transform", WINDOW_NORMAL);
				    //imshow("Morphological Transform", post_gray);

				    //// Apply convolution with Gaussian kernel for extracting the light-emitting regions
				    Mat gaussian_kernel = getGaussianKernel(5, 1.2);

				    vector<KeyPoint> keypoints;
				    keypoints = blob_detection(post_gray);
				    cout << "KEYPOINTS -----------> " << keypoints.size() << endl;

				    //namedWindow("Gaussian Convolution", WINDOW_NORMAL);
				    //imshow("Gaussian Convolution", post_gray);

				    Mat tl_candidates;
				    Mat rgb_with_candidates = upper_roi_mat.clone();
				    drawKeypoints(upper_roi_mat, keypoints, tl_candidates, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

				    vector<Point2f> candidates_coord;
				    for (auto i = 0; i < keypoints.size(); i++) {
					    float x = keypoints[i].pt.x;
					    float y = keypoints[i].pt.y;
					    candidates_coord.emplace_back(Point2f(x, y));
				    }

				    // Red Template convolution with of Gaussian Kernel
				    Mat red_fltrd_templ, red_fltrd2_templ;
				    filter2D(red_templ, red_fltrd_templ, -1, gaussian_kernel, Point(-1, -1), 5.0, BORDER_REPLICATE);
				    filter2D(red_fltrd_templ, red_fltrd2_templ, -1, gaussian_kernel, Point(-1, -1), 5.0, BORDER_REPLICATE);
				    /*namedWindow("Gaussian Convolution Template", WINDOW_NORMAL);
				    imshow("Gaussian Convolution Template", red_fltrd2_templ);*/

				    // Green Template convolution with of Gaussian Kernel
				    Mat green_fltrd_templ, green_fltrd2_templ;
				    filter2D(green_templ, green_fltrd_templ, -1, gaussian_kernel, Point(-1, -1), 5.0, BORDER_REPLICATE);
				    filter2D(green_fltrd_templ, green_fltrd2_templ, -1, gaussian_kernel, Point(-1, -1), 5.0, BORDER_REPLICATE);
				    //namedWindow("Gaussian Convolution Template", WINDOW_NORMAL);
				    //imshow("Gaussian Convolution Template", green_fltrd2_templ);

				    // Defining and drawing the bounding boxes around traffic light candidates
				    float total_red_min = 100;
				    float red_min_limit = 0.29;
				    float total_green_min = 100;
				    float green_min_limit = 0.29;
				    Point upper_left_corner;
				    Point lower_right_corner;

				    bool red_detected = false;
				    bool green_detected = false;

				    for (auto i = 0; i < candidates_coord.size(); i++) {
					    float box_width =  2 * keypoints[i].size;
					    float box_height = 2 * keypoints[i].size;

					    if (candidates_coord[i].x + box_width / 2 > upper_roi_mat.cols)
						    box_width = upper_roi_mat.cols - candidates_coord[i].x;
					    else if (candidates_coord[i].y + box_height / 2 > upper_roi_mat.rows)
						    box_height = upper_roi_mat.rows - candidates_coord[i].y;
					    else if (candidates_coord[i].x - box_width / 2 < 0)
						    candidates_coord[i].x = box_width / 2;
					    else if (candidates_coord[i].y - box_height / 2 < 0)
						    candidates_coord[i].y = box_height / 2;
					    Rect candidate_box(candidates_coord[i].x - box_width / 2, candidates_coord[i].y - box_height / 2, box_width, box_height);
					    Mat candidate_im = upper_roi_mat(candidate_box);

					    /// Separate the image in 3 places ( B, G and R )
					    vector<Mat> bgr_planes;
					    split(candidate_im, bgr_planes);
					    /// Establish the number of bins
					    int histSize = 64;
					    /// Set the ranges ( for B,G,R) )
					    float range[] = { 0, 256 };
					    const float* histRange = { range };
					    bool uniform = true; bool accumulate = false;
					    Mat b_hist, g_hist, r_hist;
					    /// Compute the histograms:
					    calcHist(&bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
					    calcHist(&bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
					    calcHist(&bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);
					    // Draw the histograms for B, G and R
					    int hist_w = 512; int hist_h = 400;
					    int bin_w = cvRound((double)hist_w / histSize);
					    Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
					    /// Normalize the result to [ 0, histImage.rows ]
					    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
					    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
					    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
					    /// Draw for each channel
					    for (int i = 1; i < histSize; i++)
					    {
						    line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
							    Point(bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
							    Scalar(255, 0, 0), 2, 8, 0);
						    line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
							    Point(bin_w*(i), hist_h - cvRound(g_hist.at<float>(i))),
							    Scalar(0, 255, 0), 2, 8, 0);
						    line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
							    Point(bin_w*(i), hist_h - cvRound(r_hist.at<float>(i))),
							    Scalar(0, 0, 255), 2, 8, 0);
					    }
					    /// Display
					    string hist = "calcHist Demo " + to_string(i);
					    //namedWindow(hist, WINDOW_NORMAL);
					    //imshow(hist, histImage);

					    // Testing the Red & Green templates
					    //
					    // RED TEMPLATE
					    resize(red_fltrd2_templ, red_templ_resized, Size(box_width / 2, box_height / 2));
					    Mat result_candidate;
					    matchTemplate(candidate_im, red_templ_resized, result_candidate, TM_SQDIFF_NORMED);
					    // Localizing the best match with minMaxLoc
					    double minVal; double maxVal; Point minLoc; Point maxLoc;
					    Point matchLoc;
					    minMaxLoc(result_candidate, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
					    matchLoc = minLoc;
					    cout << "RED MIN is ------------------->>> " << minVal << endl;
					    if (total_red_min > minVal) {
						    total_red_min = minVal;
					    }
					    if (minVal < red_min_limit) {
						    upper_left_corner = Point(candidates_coord[i].x - box_width / 2, candidates_coord[i].y - box_height / 2);
						    lower_right_corner = Point(candidates_coord[i].x + box_width / 2, candidates_coord[i].y + box_height / 2);
						    rectangle(tl_candidates, upper_left_corner, lower_right_corner, Scalar(0, 0, 255));
						    cout << "TRAFFIC LIGHT DETECTED IS ------------------->>> RED " << minVal << endl;
						    red_detected = true;
					    }
					    /*else if (r_hist.at<float>(63) > 250 && r_hist.at<float>(58) < 150) {
						    upper_left_corner = Point(candidates_coord[i].x - box_width / 2, candidates_coord[i].y - box_height / 2);
						    lower_right_corner = Point(candidates_coord[i].x + box_width / 2, candidates_coord[i].y + box_height / 2);
						    rectangle(tl_candidates, upper_left_corner, lower_right_corner, Scalar(0, 0, 255));
						    cout << "TRAFFIC LIGHT DETECTED IS ------------------->>> RED " << minVal << endl;
						    red_detected = true;
					    }*/
					    //
					    // GREEN TEMPLATE
					    resize(green_fltrd2_templ, green_templ_resized, Size(box_width / 2 , box_height / 2));
					    matchTemplate(candidate_im, green_templ_resized, result_candidate, TM_SQDIFF_NORMED);
					    // Localizing the best match with minMaxLoc
					    minMaxLoc(result_candidate, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
					    matchLoc = minLoc;
					    cout << "GREEN MIN is ------------------->>>  " << minVal << endl;
					    if (total_green_min > minVal) {
						    total_green_min = minVal;
					    }
					    if (minVal < green_min_limit) {
						    upper_left_corner = Point(candidates_coord[i].x - box_width / 2, candidates_coord[i].y - box_height / 2);
						    lower_right_corner = Point(candidates_coord[i].x + box_width / 2, candidates_coord[i].y + box_height / 2);
						    rectangle(tl_candidates, upper_left_corner, lower_right_corner, Scalar(0, 255, 0));
						    cout << "TRAFFIC LIGHT DETECTED IS ------------------->>> GREEN " << minVal << endl;
						    green_detected = true;
					    }
					    /*else if (r_hist.at<float>(63) < 100 && b_hist.at<float>(63) > 150 && g_hist.at<float>(63) > 150) {
						    upper_left_corner = Point(candidates_coord[i].x - box_width / 2, candidates_coord[i].y - box_height / 2);
						    lower_right_corner = Point(candidates_coord[i].x + box_width / 2, candidates_coord[i].y + box_height / 2);
						    rectangle(tl_candidates, upper_left_corner, lower_right_corner, Scalar(0, 0, 255));
						    cout << "TRAFFIC LIGHT DETECTED IS ------------------->>> GREEN " << minVal << endl;
						    green_detected = true;
					    }*/

					    // Just a helper display.. will be rejected in the final version
					    // --START--
					    drawing = candidate_im.clone();
					    rectangle(drawing, matchLoc, Point(matchLoc.x + red_templ_resized.cols, matchLoc.y + red_templ_resized.rows), Scalar::all(255), 1, 8, 0);
					    //namedWindow("Candidate Template Matching", WINDOW_NORMAL);
					    //imshow("Candidate Template Matching", drawing);
					    // --END--

					    /*Point upper_left_corner(candidates_coord[i].x - box_width / 2, candidates_coord[i].y - box_height / 2);
					    Point lower_right_corner(candidates_coord[i].x + box_width / 2, candidates_coord[i].y + box_height / 2);
					    rectangle(tl_candidates, upper_left_corner, lower_right_corner, Scalar(255, 100, 0));*/
				    }
                    
				    if (red_detected){
				        char msg[1024];
				        memset(msg, 0, 1024); // clearing the buffer
			            strcpy(msg,"1\n");
			            if (server > 0 and !vibrating){
			                send(server,msg,sizeof(msg),0);
			                vibrating = true;
			            }else
			                cout << "Red signal did not sent.." << endl;
					    cout << "ATTENTION! Red Light Ahead!" << endl;
				    }else if (!red_detected && green_detected){
				        char msg[1024];
				        memset(msg, 0, 1024); // clearing the buffer
			            strcpy(msg,"2\n");
			            if (server > 0 and !vibrating){
			                send(server,msg,sizeof(msg),0);
			                vibrating = true;
			            }else
			                cout << "Green signal did not sent.." << endl;
					    cout << "Please Walk Ahead, Green Light" << endl;
				    }else if (!red_detected && !green_detected){
				        char msg[1024];
				        memset(msg, 0, 1024); // clearing the buffer
			            strcpy(msg,"3\n");
			            if (server > 0 and !vibrating){
			                send(server,msg,sizeof(msg),0);
			                vibrating = true;
			            }else
			                cout << "Unknown signal did not sent.." << endl;
					    cout << "!@#$%^& Cannot Decide.... It's at your own risk!" << endl;
                    }
				    //namedWindow("Candidates", WINDOW_NORMAL);
				    //imshow("Candidates", tl_candidates);

				    ///*namedWindow("Mask", WINDOW_NORMAL);
				    //imshow("Mask", res_dest);*/

				    /********************************************** End **********************************************/

				    //imshow("Image", upper_roi_mat);
				    //imshow("Colormap", total_pixel_score);

				    // Release all Mat files untill now
				    total_pixel_score.release();
				    //roiMat.release();
				    upper_roi_mat.release();
				    rgb_im.release();
				    //dth_fltred_im.release();
                }
			}

		}
		catch (const rs2::error & e) 
		{
			std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
			//dev.hardware_reset();
			//rs2::device_hub hub(ctx);
			//dev = hub.wait_for_device(); //Note that device hub will get any device, if you have more than 1 connected it could return the other device
		}

		auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
		auto intrinsics = stream.get_intrinsics(); // Calibration data
		
		//min_vibration_delay = fps_counter.get_current_fps();
		if (vibrating){
            vibration_delay += 1;
            if (vibration_delay > min_vibration_delay){     // wait for min_vibration_delay frames until send a new vibration message
				vibrating = false;
                vibration_delay = 0;
            }
        }

	}


	cout << "Closing App..." << endl;

	// Signal threads to finish and wait until they do
	processing_thread.join();
	pipe.stop();

	return EXIT_SUCCESS;


}

catch (const rs2::error & e) 
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e) 
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

bool is_Box_Area_Clear(Mat _roi) 
{
	Rect roi(0, 0.1 * _roi.rows, _roi.cols, 0.8 * _roi.rows);
	Mat box = _roi(roi);

	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 190;
	params.maxThreshold = 255;

	// Filter by Color.
	params.filterByColor = false;
	params.blobColor = 255;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 200;

	// Filter by Circularity
	params.filterByCircularity = false;
	params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = false;
	params.minConvexity = 0.87;

	// Filter by Inertia
	params.filterByInertia = false;
	params.minInertiaRatio = 0.01;

	// Set up detector with params
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	// SimpleBlobDetector::create; //creates a smart pointer. 
	// So you need to use arrow ( ->) instead of dot ( . )
	std::vector<KeyPoint> keypoints;
	detector->detect(box, keypoints);

	Mat im_with_keypoints;
	drawKeypoints(box, keypoints, im_with_keypoints, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	// Show blobs
	//namedWindow("keypoints", WINDOW_NORMAL);
	//imshow("keypoints", im_with_keypoints);

	if (keypoints.size() > 0)
		return false;
	else
		return true;
}

vector<KeyPoint> blob_detection(Mat source_im)
{
	Mat box = source_im.clone();

	// Setup SimpleBlobDetector parameters
	SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 110;
	params.maxThreshold = 255;

	// Filter by Color.
	params.filterByColor = false;
	params.blobColor = 255;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 5;
	params.maxArea = 60;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.2;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.2;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.2;
	params.maxInertiaRatio = 0.85;

	// Set up detector with params
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

	// SimpleBlobDetector::create; //creates a smart pointer. 
	// So you need to use arrow ( ->) instead of dot ( . )
	std::vector<KeyPoint> keypoints;
	detector->detect(box, keypoints);

	if (keypoints.size() > 1) {
		cout << "FOUND " << keypoints.size() << " Keypoints" << endl;
	}

	Mat im_with_keypoints;
	drawKeypoints(box, keypoints, im_with_keypoints, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	// Show blobs
	//namedWindow("TL Candidates", WINDOW_NORMAL);
	//imshow("TL Candidates", im_with_keypoints);
	
	return keypoints;
}





