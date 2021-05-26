#include <imagePipeline.h>
#include <iostream>
#include "opencv2/core.hpp"
//#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <cmath>
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        // img is the scene input from the camera
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/

        //Store the confidence of each template against the scene
        int confidence [boxes.templates.size()];
        float maxConfidence = 0.0;
        int conf_threshold = 25;
        // Set the scene as one of the tag images (e.g. "tag_2", "tag_9" etc. up to 15, "tag_blank")
        //Mat img_scene = cv::imread("mie443_contest2/boxes_database/tag_2", CV_LOAD_IMAGE_GRAYSCALE);
        //Set the scene from camera input
        Mat img_scene = img;
        // Convert to grayscale
        cvtColor(img_scene, img_scene, COLOR_RGB2GRAY);
        Mat img_obj;
        int minHessian = 800;
        Ptr<SURF> detector = SURF::create(minHessian);
        const float ratio_thresh = 0.7f;

        //Detect keypoints in scene
        std::vector<KeyPoint> kypt_scene;
        Mat des_scene;
        detector -> detectAndCompute(img_scene, noArray(), kypt_scene, des_scene);

        Ptr <DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

        // Check against all possible box templates
        for (int i = 0; i < boxes.templates.size(); i++){
            img_obj = boxes.templates[i];   
            cv::resize(img_obj, img_obj, cv::Size(500,400)); //Match template image ratio to box images     

            //lines below from this tutorial: https://docs.opencv.org/3.4.4/d7/dff/tutorial_feature_homography.html
            
            // Detect object keypoints and descriptors with SURF
            std::vector<KeyPoint> kypt_obj;
            Mat des_obj;
            detector -> detectAndCompute(img_obj, noArray(), kypt_obj, des_obj);
            
            // Use knn for feature matching, L2 norm
            std::vector< std::vector<DMatch> > knn_matches;
            // Prevent openCV error - check for empty scene descriptor matrix (rare cases where robot looks into nowhere or wall)
            if (des_scene.empty()){
                std::cout << "empty scene descriptor matrix, skip to next box!" << std::endl;
                continue;
            }
            matcher->knnMatch(des_obj, des_scene, knn_matches, 2);

            //Filter best matches only           
            std::vector<DMatch> good_matches;
            for (size_t i=0; i < knn_matches.size(); i++){
                if (knn_matches[i][0].distance < ratio_thresh*knn_matches[i][1].distance){
                    good_matches.push_back(knn_matches[i][0]);
                }
            }
            std::cout << "tag number: " << i+1 << " -> Confidence: " << good_matches.size() << std::endl;
            
            // draw homography box, check the boundary area
            Mat img_matches;
            drawMatches(img_obj, kypt_obj, img_scene, kypt_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            std::vector<Point2f> obj;
            std::vector<Point2f> scene;

            for( size_t i =0; i<good_matches.size(); i++){
                obj.push_back(kypt_obj[good_matches[i].queryIdx].pt);
                scene.push_back(kypt_scene[good_matches[i].trainIdx].pt);
            }
            
            // Prevent bad argument error
            if (obj.size() == 0 || scene.size() == 0){
            //if (obj.size() < 4){
                std::cout << "empty obj and scene vectors, skip to next image!" << std::endl;
                continue;
            }

            // Find homography
            Mat H = findHomography(obj, scene, RANSAC);
            // Prevent assertion error
            if (H.empty()){
                std::cout << "empty H matrix, skip to next image!" << std::endl;
                continue;
            }
            
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = Point2f(0,0);
            obj_corners[1] = Point2f( (float)img_obj.cols, 0);
            obj_corners[2] = Point2f( (float)img_obj.cols, (float)img_obj.rows);
            obj_corners[3] = Point2f(0, (float)img_obj.rows);
            
            std::vector<Point2f> scene_corners(4);

            perspectiveTransform(obj_corners, scene_corners, H);

            line( img_matches, scene_corners[0] + Point2f((float)img_obj.cols, 0), scene_corners[1] + Point2f((float)img_obj.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f((float)img_obj.cols, 0), scene_corners[2] + Point2f((float)img_obj.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f((float)img_obj.cols, 0), scene_corners[3] + Point2f((float)img_obj.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f((float)img_obj.cols, 0), scene_corners[0] + Point2f((float)img_obj.cols, 0), Scalar( 0, 255, 0), 4 );
            
            // Calculate the area of the green box formed using distances between the points
            Point2f point1 = scene_corners[0] + Point2f((float)img_obj.cols, 0);
            Point2f point2 = scene_corners[1] + Point2f((float)img_obj.cols, 0);
            Point2f point3 = scene_corners[2] + Point2f((float)img_obj.cols, 0);
            Point2f point4 = scene_corners[3] + Point2f((float)img_obj.cols, 0);
            float greenBoxSize = fabs((point1.x*point2.y - point1.y*point2.x) + (point2.x*point3.y - point2.y*point3.x) + (point3.x*point4.y - point3.y*point4.x) + (point4.x*point1.y - point4.y*point1.x))/2;
            // Display the keypoint matches
            float scale_percent = 0.4;
            Mat small_matches;
            resize(img_matches, small_matches, Size(img_matches.cols*scale_percent, img_matches.rows*scale_percent), 0.5, 0.5, CV_INTER_LINEAR);
            imshow("Keypoint detection", small_matches);
            std::cout << "greenBoxSize: " << greenBoxSize << std::endl;
            //Update most likely template_id if it has most matches & better than some threshold
            //if (good_matches.size() > maxConfidence && good_matches.size() > conf_threshold){
            if (good_matches.size() > maxConfidence && good_matches.size() > conf_threshold && greenBoxSize > 80000 && greenBoxSize < 375000){
                maxConfidence = good_matches.size();
                template_id = i;
                std::cout << "maxConfidence updated" << std::endl;
                std::cout << "New best tag is: " << i+1 << std::endl;
            }
            cv::waitKey(10);
        }
        // Unless determined as blank image, display the best image match
        if (template_id > -1){
            imshow("Best template match", boxes.templates[template_id]);
        }
        else if (template_id == -1){
            std::cout << "empty H matrices, blank image!" << std::endl;
        }
        cv::waitKey(10);
    }  
    return template_id;
}

