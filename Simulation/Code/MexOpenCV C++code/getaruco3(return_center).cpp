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
    int sizearr[2];
    if(markerCorners.size()==1)
    {
        cv::Point2f center(0, 0);
        for (size_t j = 0; j < 4; j++) {
            center += markerCorners[0][j];
        }
        center*=0.25;
        cv::Point centerInt(cvRound(center.x), cvRound(center.y));
        sizearr[0]=centerInt.x;
        sizearr[1]=centerInt.y;
    }

    plhs[0]=mxCreateNumericMatrix(1,2,mxINT32_CLASS,mxREAL);
    int *outputarray=(int*)mxGetData(plhs[0]);
    outputarray[0]=sizearr[0];
    outputarray[1]=sizearr[1];
}   