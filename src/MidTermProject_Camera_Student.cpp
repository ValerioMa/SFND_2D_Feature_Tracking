/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <list>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

#include <fstream>      // std::fstream

using namespace std;

// #define LOG_DATA
// #define LOOP
/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    
    /* INIT VARIABLES AND DATA STRUCTURES */
#ifdef LOOP    
    std::vector<string> detectorTypes = {"FAST", "SHITOMASI", "ORB", "BRISK", "SIFT", "HARRIS", "AKAZE"}; //, "SHITOMASI", "AKAZE", "ORB", "FAST", "BRISK", "SIFT", "HARRIS"};
    std::vector<string> kpDescriptorTypes = {"ORB", "FREAK", "SIFT", "BRISK", "BRIEF", "AKAZE"};
    
    for(const string detectorType : detectorTypes){
        for(const string kpDescriptorType : kpDescriptorTypes){
#else
    const string kpDescriptorType = "SIFT";
    const string detectorType = "FAST";
#endif

            if ((kpDescriptorType.compare("AKAZE") == 0 && detectorType.compare("AKAZE") != 0) ||
            (detectorType.compare("SIFT") == 0 && kpDescriptorType.compare("ORB") == 0))
            {
                // AKAZE descriptors can only be used with KAZE or AKAZE keypoints.
                // ORB descriptors are not compatible with SIFT detetor
                #ifdef LOOP 
                    continue;
                #else
                    return -1;
                #endif
            }

            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            if(kpDescriptorType.compare("SIFT")==0){
                descriptorType = "DES_HOG";
            }
            const string selectorType = "SEL_KNN";      // SEL_NN, SEL_KNN            
            const string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN            

#ifdef LOG_DATA
            const string fileName = "./data/" + detectorType + "_" + kpDescriptorType + ".txt";
            std::cout << "#0: LOG FILE " << fileName << std::endl;
            std::fstream fs;
            fs.open (fileName, std::fstream::out);
            fs << " nKp, tKp, nKpV, tDesc, nMatch, tMatch, Atot, AUnion | Kp size;";
#endif

            // data location
            string dataPath = "../";

            // camera
            string imgBasePath = dataPath + "images/";
            string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
            string imgFileType = ".png";
            int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
            int imgEndIndex = 9;   // last file index to load
            int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

            // misc
            int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
            std::list<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
            bool bVis = false;            // visualize results

            // Init detector and descriptors            
            descKeypointsInit(kpDescriptorType);            
            if(!(detectorType.compare("HARRIS") == 0 || detectorType.compare("SHITOMASI")== 0)){                
                detKeypointsModernInit(detectorType);
            }
            
            

            /* MAIN LOOP OVER ALL IMAGES */
            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
#ifdef LOG_DATA
                fs << "\n";
#endif                

                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                //// STUDENT ASSIGNMENT
                //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

                // push image into data frame buffer
                if(dataBuffer.size() >= dataBufferSize){
                    dataBuffer.pop_front();
                }
                DataFrame frame;
                frame.cameraImg = imgGray;
                dataBuffer.push_back(frame);
                const auto last = dataBuffer.rbegin();

                //// EOF STUDENT ASSIGNMENT                
                cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image        
                bool bVis = false;

                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
#ifdef LOG_DATA
                double t = (double)cv::getTickCount();
#endif                
                if (detectorType.compare("SHITOMASI") == 0)  
                {
                    detKeypointsShiTomasi(keypoints, imgGray, bVis);
                }
                else if(detectorType.compare("HARRIS") == 0)
                {
                    detKeypointsHarris(keypoints, imgGray, bVis);
                }else{
                    detKeypointsModern(keypoints, imgGray, bVis);
                }
#ifdef LOG_DATA
                t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
                fs << keypoints.size() << ", " << t << ", ";
#endif                
                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                cv::Rect vehicleRect(535, 180, 180, 150);
                if (bFocusOnVehicle)
                {
                    keypoints.erase(std::remove_if(keypoints.begin(), 
                                    keypoints.end(),
                                    [vehicleRect](cv::KeyPoint x){return !vehicleRect.contains(x.pt);}),
                                    keypoints.end());
                    
                }
#ifdef LOG_DATA
                fs << keypoints.size() << ", ";
#endif                
                //// EOF STUDENT ASSIGNMENT

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;

                    if (detectorType.compare("SHITOMASI") == 0)
                    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << " NOTE: Keypoints have been limited!" << endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                last->keypoints = keypoints;
                cout << "#2 : DETECT KEYPOINTS done" << endl;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;   
#ifdef LOG_DATA
                double t1 = (double)cv::getTickCount();
#endif
                descKeypoints(last->keypoints, last->cameraImg, descriptors);
#ifdef LOG_DATA
                t1 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
                fs << t1 << ", "; 
#endif

                //// EOF STUDENT ASSIGNMENT

                // push descriptors for current frame to end of data buffer
                last->descriptors = descriptors;

                cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {

                    auto prev = dataBuffer.rbegin();
                    prev++;
                    
                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                
                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
#ifdef LOG_DATA
                    double t2 = (double)cv::getTickCount();
#endif
                    matchDescriptors(prev->keypoints, last->keypoints,
                                    prev->descriptors, last->descriptors,
                                    matches, descriptorType, matcherType, selectorType);
#ifdef LOG_DATA
                    t2 = ((double)cv::getTickCount() - t2) / cv::getTickFrequency();
                    fs << matches.size() << ", " << t2 << ", ";
#endif
                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    last->kptMatches = matches;

                    cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                    // visualize matches between current and previous image
                    bVis = true;
                    if (bVis)
                    {
                        cv::Mat matchImg = (last->cameraImg).clone();
                        cv::drawMatches(prev->cameraImg, prev->keypoints,
                                        last->cameraImg, last->keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        cout << "Press key to continue to next image" << endl;
                        cv::waitKey(); // wait for key to be pressed
                    }

#ifdef LOG_DATA
                    // COMPUTE UNION AND INTERSECTION OF KP AREA
                    const int draw_shift_bits = 4;
                    const int draw_multiplier = 1 << draw_shift_bits;
                    cv::Mat intersection_area(imgGray.size(), CV_8U, cv::Scalar(0));
                    double area_tot = 0;
                    for(const auto p : last->keypoints){
                        area_tot += (p.size*p.size)/4.*M_PI;
                        cv::Point center( cvRound(p.pt.x * draw_multiplier), cvRound(p.pt.y * draw_multiplier) );
                        int radius = cvRound(p.size/2 * draw_multiplier); // KeyPoint::size is a diameter
                        cv::circle( intersection_area, center, radius, cv::Scalar(255), -1, cv::FILLED, draw_shift_bits );
                    }

                    std::vector<std::vector<cv::Point> > contours;
                    cv::findContours(intersection_area, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);        
                    //cv::drawContours(intersection_area, contours, -1, cv::Scalar(125), -1, cv::LINE_AA);

                    int area =  countNonZero(intersection_area);

                    fs << area_tot << ", " << area; 
                    fs << "| ";
                    for(const auto p : last->keypoints){
                        fs << p.pt.x << ", " << p.pt.y << ", " << p.size << "; ";                
                    }            
                    
#endif
                }

            } // eof loop over all images    
#ifdef LOG_DATA                 
            fs.close();
#endif

#ifdef LOOP 
        }
    }
#endif    
    return 0;
}
