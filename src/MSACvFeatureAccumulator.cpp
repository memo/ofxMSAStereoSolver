/*
 *  FeatureAccumulator.cpp
 *  Stereo Converter
 *
 *  Created by Memo Akten on 19/08/2010.
 *  Copyright 2010 MSA Visuals Ltd. All rights reserved.
 *
 */

#include "MSACvFeatureAccumulator.h"


namespace MSA {
    namespace Cv {
        
        //----------------------------------------
        void FeatureAccumulator::clear() {
            objectPoints.clear();
            imagePoints.clear();
        }
        
        
        //----------------------------------------
        void FeatureAccumulator::addFeatures(Features& f) {
            if(f.getNumPoints() > 0) {
                objectPoints.push_back(f.objectPoints);
                imagePoints.push_back(f.imagePoints);
            } else {
                ofLog(OF_LOG_WARNING, "FeatureAccumulator::addFeatures - objectPoint count doesn't match imagePoint count");
            }
        }
        
        
        //----------------------------------------
        int FeatureAccumulator::getNumImages() {
            int n1 = objectPoints.size();
            int n2 = imagePoints.size();
            if(n1!=n2) {
                ofLog(OF_LOG_WARNING, "FeatureAccumulator::getNumImages - number of object points + " + ofToString(n1) + " doesn't match number of image points " + ofToString(n2));
            }
            
            return n1;
        }
        
    }
}