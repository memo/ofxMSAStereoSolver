/*
 *  FeatureFinderBase.h
 *  Stereo Solver
 *
 *  Created by Memo Akten on 14/08/2010.
 *  Copyright 2010 MSA Visuals Ltd. All rights reserved.
 *
 */

#pragma once

#include "MSACvFeatures.h"


namespace MSA {
    namespace Cv {
        
        class FeatureFinderBase {
        public:
            virtual ~FeatureFinderBase() {}
            virtual int find(ofxCvGrayscaleImage &sourceImage) = 0;
            virtual void clear() = 0;
            
            Features& getFeatures() {
                return _features;
            }
            
        protected:
            Features _features;
        };
        
    }
}