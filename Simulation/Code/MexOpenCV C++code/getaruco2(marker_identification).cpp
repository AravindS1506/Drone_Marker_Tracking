#include "opencvmex.hpp"
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;  

//////////////////////////////////////////////////////////////////////////////
// Check inputs
//////////////////////////////////////////////////////////////////////////////
void checkInputs(int nrhs, const mxArray *prhs[])
{
    if (nrhs != 2)
    {
        mexErrMsgTxt("Incorrect number of inputs. Function expects 2 scalar inputs.");
    }
}

///////////////////////////////////////////////////////////////////////////
// Main entry point to a MEX function
///////////////////////////////////////////////////////////////////////////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    
    cv::Mat img;
    ocvMxArrayToImage_uint8(prhs[0], img);
    int dictionaryId = (int)mxGetScalar(prhs[1]);
    


    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    aruco::detectMarkers(img, dictionary,markerCorners, markerIds,detectorParams,rejectedCandidates);
    aruco::drawDetectedMarkers(img, markerCorners, markerIds);
    
    plhs[0] = ocvMxArrayFromImage_uint8(img);
}