/*
 *  FeatureFinderShiTomasi.h
 *  Stereo Converter
 *
 *  Created by Memo Akten on 18/08/2010.
 *  Copyright 2010 MSA Visuals Ltd. All rights reserved.
 *
 */

#pragma once

#include "MSACvFeatureFinderBase.h"

namespace MSA {
    namespace Cv {
        
        class FeatureFinderShiTomasi : public FeatureFinderBase {
        public:
            
            // input params, can be set at runtime, after allocate
            struct  {
                int		maxCornerCount;
                float	qualityLevel;		// should not exceed 1 (a typical value might be 0.10 or 0.01)
                float	minDistance;		// no two returned points are within the indicated number of pixels
                int		blockSize;			// must be odd
                bool	useHarris;			// Harris corner definition is used rather than the Shi-Tomasi definition
                float	harrisWeight;		// (only for harris) weighting coefficient used to set the relative weight given to the trace of the autocorrelation matrix Hessian compared to the determinant of the same matrix
            } settings;
            
            
            FeatureFinderShiTomasi();
            void allocate(int w, int h);    // w, h dimensions of image
            void clear();
            int find(ofxCvGrayscaleImage &sourceImage);
            

        protected:
            ofxCvFloatImage			_tempImage;
            ofxCvFloatImage			_eigImage;
            vector<CvPoint2D32f>	_tempImagePoints;
            
        };
        
    }
}