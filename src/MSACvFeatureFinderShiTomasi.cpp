/*
 *  FeatureFinderShiTomasi.cpp
 *  Stereo Converter
 *
 *  Created by Memo Akten on 18/08/2010.
 *  Copyright 2010 MSA Visuals Ltd. All rights reserved.
 *
 */

#include "MSACvFeatureFinderShiTomasi.h"

namespace MSA {
    namespace Cv {
        
        //----------------------------------------
        FeatureFinderShiTomasi::FeatureFinderShiTomasi() {
            settings.maxCornerCount		= 100;
            settings.qualityLevel		= 0.1;
            settings.minDistance		= 10;
            settings.blockSize			= 3;
            settings.useHarris			= false;
            settings.harrisWeight		= 4;
        }
        
        
        //----------------------------------------
        void FeatureFinderShiTomasi::allocate(int w, int h) {
            _tempImage.allocate(w, h);
            _eigImage.allocate(w, h);
        }
        
        
        //----------------------------------------
        void FeatureFinderShiTomasi::clear() {
            _features.clear();
        }
        
        
        //----------------------------------------
        int FeatureFinderShiTomasi::find(ofxCvGrayscaleImage &sourceImage) {
            _features.numFeaturesFound = settings.maxCornerCount;
            if(_tempImagePoints.size() != settings.maxCornerCount) _tempImagePoints.resize(settings.maxCornerCount);
            
            cvGoodFeaturesToTrack(sourceImage.getCvImage(), _tempImage.getCvImage(), _eigImage.getCvImage(),
                                  &_tempImagePoints[0], &_features.numFeaturesFound, settings.qualityLevel, settings.minDistance, 
                                  NULL, settings.blockSize, settings.useHarris, settings.harrisWeight);
            
            _features.imagePoints.clear();
            
            if(_features.numFeaturesFound) {
                _features.imagePoints.insert(_features.imagePoints.begin(), _tempImagePoints.begin(), _tempImagePoints.begin() + _features.numFeaturesFound);
                cvFindCornerSubPix(sourceImage.getCvImage(), &_features.imagePoints[0], _features.numFeaturesFound, cvSize(11, 11), cvSize(-1, -1), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
            } else {
                ofLog(OF_LOG_VERBOSE, "MSA::Cv::FeatureFinderShiTomasi::find - no features found");
            }
            
            return _features.numFeaturesFound;
        }
        
    }
}