/*
 *  FeatureFinderBase.h
 *  Stereo Converter
 *
 *  Created by Memo Akten on 19/08/2010.
 *  Copyright 2010 MSA Visuals Ltd. All rights reserved.
 *
 */

#pragma once


#include "ofMain.h"
#include "opencv2/opencv.hpp"


namespace MSA {
    namespace Cv {
        
        class Features {
        public:
            
            // making these public, don't have time for full API right now, use carefully!
            vector<cv::Point3f>	objectPoints;       // coordinates in space
            vector<cv::Point2f>	imagePoints;        // coordinates in image
            
            
            //----------------------------------------
            void clear() {
                objectPoints.clear();
                imagePoints.clear();
            }
            
            
            //----------------------------------------
            void draw(float x, float y) {
                // TODO: make this vertex array or VBO
                glBegin(GL_POINTS);
                for(int i=0; i<imagePoints.size(); i++) {
                    glVertex2f(imagePoints[i].x + x, imagePoints[i].y + y);
                }
                glEnd();
            }

            
            //----------------------------------------
            const int getNumPoints() {
                int n1 = objectPoints.size();
                int n2 = imagePoints.size();
                if(n1!=n2) {
                    ofLog(OF_LOG_WARNING, "Features::getNumPoints - number of object points + " + ofToString(n1) + " doesn't match number of image points " + ofToString(n2));
                    return 0;
                }
                
                return n1;
            }
            
        };

    }
}
