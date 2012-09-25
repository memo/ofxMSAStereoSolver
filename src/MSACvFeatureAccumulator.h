/*
 *  FeatureAccumulator.h
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
        
        class FeatureAccumulator {
        public:
            // making these public, don't have time for full API right now, use carefully!
            // keeping these separate arrays (not interleaved) so they can be sent to opencv straight away
            vector<vector<cv::Point3f> > objectPoints;
            vector<vector<cv::Point2f> > imagePoints;
            
            void clear();
            void addFeatures(Features& f);
            int getNumImages();
            
        };
        
    }
}