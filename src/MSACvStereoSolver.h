/*
 *  StereoSolver.h
 *  Stereo Converter
 *
 *  Created by Memo Akten on 14/08/2010.
 *  Copyright 2010 MSA Visuals Ltd. All rights reserved.
 *
 */


#pragma once

#include "ofMain.h"
#include "opencv2/opencv.hpp"
//#include "MSACvFeatureAccumulator.h"
//#include "MSACvFeatureFinderChessboard.h"


namespace MSA {
    namespace Cv {
        
        class StereoSolver {
        public:
            StereoSolver();
//            ~StereoSolver();
            
            enum DisparityMethod { STEREO_BM, STEREO_SGBM, STEREO_VAR };
            
            // input params, can be set at runtime, after allocate
            struct {
                struct {
                    bool            useCalibrated;      // BOUGUET'S METHOD vs HARTLEY'S METHOD
                    float           squareSize;
                    int             grabTime;           // how many seconds between grabbing a frame for auto-calibation
                    int             totalImageCount;          // how many frames to grab for auto-calibration
                    bool            doDrawLines;        // draw horizontal lines
                } calibration;
                
                struct {
                    DisparityMethod	disparityMethod;
                    float			depthMin;
                    float			depthMax;
                    bool			depthNormalize;
                } stereo;
            } settings;
            
            
            cv::Mat     origMat[2];    // reference to original images passed in (needed for drawing only)
            cv::Mat		rectMat[2];    // rectified images
            cv::Mat		dispMat;       // disparity
            cv::Mat		depthMat;      // depth
            
            ofImage     *origImage[2];
            ofTexture     rectImage[2];
            ofTexture     dispImage;
            
            
            // these are public so you can directly modify all properties, instead of me building a large API to access them
            cv::StereoBM    bm;
            cv::StereoSGBM  sgbm;
            cv::StereoVar   var;
            
            
            //----------------------------------------
//            void allocate(int w, int h);
            
            
            //----------------------------------------
            // automatic calibration and rectification using a chessboard (or saved data)

            // start looking for chessboard
            void startCalibrating(int numx, int numy, float squareSize);
            
            // stop looking for chessboard, and calculate calibration / rectification parameters
            void finishCalibrating();
            
            // save calibration data to file
            void saveCalibration(string filename = "settings/StereoCalibration.yml");
            
            // load calibration data from file
            void loadCalibration(string filename = "settings/StereoCalibration.yml");
            
            // are we currently in calibration mode?
            bool isCalibrating();
            
            // if you know input images are already rectified, set this to true. Calibration is then not nessecary
            void assumeRectifiedInput(bool b);
            
            
            
            //----------------------------------------
            // advanced (manual) calibration and rectification
            // use only if you know what you are doing
            // these are internally called by the automatic routines above
            
            // use this to find the fundamental matrix, not really nessecary if you use the other methods below
            void findFundamentalMatrix(vector<CvPoint2D32f>&p0, vector<CvPoint2D32f>&p1);
            
            // find calibration parameters given a list of features 
            void findCalibrationParams();//const FeatureAccumulator& featureAccumulator0, const FeatureAccumulator& featureAccumulator1);
            
            // find rectification parameters and precompute rectification maps
            void findRectifyParams();
            
            void calculateRectifyMaps();
            
            
            
            //----------------------------------------
            // for stereo solving
            
            // takes a stereo image pair (doesn't have to be rectified)
            // rectifies if nessecary
            // finds disparity
            // finds depth
            // if calibration mode is active, looks for chessboard and accumulates features
            bool update(ofImage& _img0, ofImage& _img1);


            void draw();
            void draw(float x, float y, float h);
            
        protected:
//            FeatureAccumulator          featureAccumulator[2];
//            FeatureFinderChessboard     chessboardFinder[2];
            
            bool					_isCalibrated;
//            bool                    _isAllocated;
            bool                    _isCalibrating;
            bool                    _assumeRectifiedInput;
            

            // for calibration
            float                           lastCalibrationGrabTime;
            int                             curGrabCount;
            cv::Size                        boardSize;
            vector<vector<cv::Point2f> >    imagePoints[2];
            vector<vector<cv::Point3f> >    objectPoints;
            
            // calibration
            cv::Mat _M[2];      // camera matrices
            cv::Mat _D[2];      // distortion coefficients for camera

            cv::Mat _R;         // relative rotation between cameras
            cv::Mat _T;         // relative translation between cameras
            cv::Mat _E;         // essential matrix
            cv::Mat _F;         // fundamental matrix
            
            // rectification
            cv::Mat _RR[2];    // 3x3 rectification transform (rotation matrix) for camera
            cv::Mat _RP[2];    // 3x4 projection matrix in the new (rectified) coordinate systems for the first camera
            cv::Mat _Q;       // 4x4 disparity-to-depth mapping matrix (see reprojectImageTo3D()).

            // warping
            cv::Mat _rmap[2][2];    // rectification maps
            
            


//            void clear();
            
            // takes raw stereo image pair (not rectified) and outputs rectifies images
            bool rectifyImages(const cv::Mat& _img0, const cv::Mat& _img1, cv::Mat& _rimg0, cv::Mat& _rimg1);
            
            // takes rectified stereo image pair and finds disparity
            bool findDisparity(const cv::Mat& _rimg0, const cv::Mat& _rimg1, cv::Mat& _dimg);
            
            bool findDepthMap();
            

        };
        
    }
}
