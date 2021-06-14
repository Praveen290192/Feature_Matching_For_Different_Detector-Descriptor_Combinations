#include <numeric>
#include "matching2D.hpp"

using namespace std;


// Find best matches for keypoints in two camera images based on several matching methods
float matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    double t = (double)cv::getTickCount();

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType;
        if (descriptorType.compare("DES_HOG") == 0)
        {
            normType = cv::NORM_L2;
        }
        else
        {
            normType = cv::NORM_HAMMING2;

        }
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if ((descSource.type() != CV_32F)|| (descRef.type() != CV_32F))
        {
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

       vector<vector<cv::DMatch>> knn_matches;
       int k = 2;
       matcher->knnMatch(descSource, descRef, knn_matches, k);
       double minDescDistRatio = 0.8;
       for (auto knnM: knn_matches)
       {
           if(knnM[0].distance < (knnM[1].distance * minDescDistRatio))
           {
               matches.push_back(knnM[0]);
           }
       }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << " Match Type:"<< matcherType << ", Descriptor Type:" << descriptorType << ", Selector Type:"<< selectorType << " "<< matches.size() <<" mathces in " << 1000 * t / 1.0 << " ms" << endl;

}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
float descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    float computationTime;
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if(descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(32);
    }
    else if(descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if(descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if(descriptorType.compare("AKAZE") == 0)
    {        
        extractor = cv::AKAZE::create();
    }
    else if(descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::xfeatures2d::SiftDescriptorExtractor::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
    computationTime = 1000 * t / 1.0;
    return computationTime;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
float detKeypointsModern(vector<cv::KeyPoint> &keypoints, cv::Mat &img, string detectorType, bool bVis )
{
    cv::Ptr<cv::FeatureDetector> detector;
    float computationTime;
    
    if (detectorType.compare("SHITOMASI") == 0)
    {
        int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
        double maxOverlap = 0.0; // max. permissible overlap between two features in %
        double minDistance = (1.0 - maxOverlap) * blockSize;
        int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

        double qualityLevel = 0.01; // minimal accepted quality of image corners
        double k = 0.04;

        // Apply corner detection
        double t = (double)cv::getTickCount();
        vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

        // add corners to result vector
        for (auto it = corners.begin(); it != corners.end(); ++it)
        {

            cv::KeyPoint newKeyPoint;
            newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
            newKeyPoint.size = blockSize;
            keypoints.push_back(newKeyPoint);
        }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << detectorType <<" detection witn n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
        computationTime = 1000 * t / 1.0;
    }
    else 
    {
        if (detectorType.compare("HARRIS") == 0)
        {
            detector = cv::xfeatures2d::HarrisLaplaceFeatureDetector::create();
            
        }
        else if (detectorType.compare("FAST") == 0)
        {
            int threshold = 30;
            bool bNMS = true;
            cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
            detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
        }
        else if (detectorType.compare("BRISK") == 0)
        {
            detector = cv::BRISK::create();//threshold, octaves, patternScale);
        }
        else if (detectorType.compare("ORB") == 0)
        {
            detector = cv::ORB::create();
        }
        else if (detectorType.compare("AKAZE") == 0)
        {
            detector = cv::AKAZE::create();
        }
        else if (detectorType.compare("SIFT") == 0)
        {
            detector = cv::xfeatures2d::SIFT::create();      
        }// compute detector parameters based on image size 
        double t = (double)cv::getTickCount();
        detector->detect(img, keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << detectorType <<" detection witn n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
        computationTime = 1000 * t / 1.0;
    
    }
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + "Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return computationTime;
}

