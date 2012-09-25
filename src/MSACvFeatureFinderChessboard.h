/*
 *  FeatureFinderChessboard.h
 *  Stereo Converter
 *
 *  Created by Memo Akten on 19/08/2010.
 *  Copyright 2010 MSA Visuals Ltd. All rights reserved.
 *
 */

#pragma once

#include "MSACvFeatures.h"

namespace MSA {
    namespace Cv {
        
        class FeatureFinderChessboard {
        public:
            
            // input params, can be set at runtime, after allocate
            struct  {
                float squareSize;
                int maxScale;
            } settings;
            
            FeatureFinderChessboard();
            void allocate(int numx, int numy); // nx, ny: number of squares on each axis
            void clear();
            int find(cv::Mat& img);
            
            
            Features& getFeatures() {
                return _features;
            }

        protected:
            cv::Size    _boardSize;
            Features    _features;


        };
    }
}
