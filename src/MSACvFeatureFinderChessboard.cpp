/*
 *  FeatureFinderChessboard.cpp
 *  Stereo Converter
 *
 *  Created by Memo Akten on 19/08/2010.
 *  Copyright 2010 MSA Visuals Ltd. All rights reserved.
 *
 */

#include "MSACvFeatureFinderChessboard.h"

namespace MSA {
    namespace Cv {
        
        //----------------------------------------
        FeatureFinderChessboard::FeatureFinderChessboard() {
            settings.squareSize = 1;
            settings.maxScale = 1;
        }

        
        //----------------------------------------
        void FeatureFinderChessboard::allocate(int numx, int numy) {
            _boardSize = cv::Size(numx, numy);
            clear();
        }
        
        
        //----------------------------------------
        void FeatureFinderChessboard::clear() {
            _features.clear();
            
            _features.imagePoints.resize(_boardSize.width * _boardSize.height);
            
            for(int j=0; j<_boardSize.height; j++)
                for(int i=0; i<_boardSize.width; i++)
                    _features.objectPoints.push_back(cv::Point3f(j*settings.squareSize, i*settings.squareSize, 0));
        }
        
        
        //----------------------------------------
        int FeatureFinderChessboard::find(cv::Mat& img) {
            bool found = false;
            for(int scale=1; scale<=settings.maxScale; scale++) {
                cv::Mat timg;
                if(scale==1) {
                    timg = img;
                } else {
                    cv::resize(img, timg, cv::Size(), scale, scale);
                }
                found = cv::findChessboardCorners(timg, _boardSize, _features.imagePoints, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
                
                if(found) {
                    if(scale > 1) {
                        cv::Mat cornersMat(_features.imagePoints);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }

            if(found) {
                cv::cornerSubPix(img, _features.imagePoints, cvSize(11, 11), cvSize(-1, -1), cv::TermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.1 ));
            } else {
                ofLog(OF_LOG_VERBOSE, "MSA::Cv::FeatureFinderChessboard::find - no features found");
                return 0;
            }
            
            return _features.getNumPoints();
        }
        


    }
}