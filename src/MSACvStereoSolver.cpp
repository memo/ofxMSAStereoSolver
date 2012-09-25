/*
 *  StereoSolver.cpp
 *  Stereo Converter
 *
 *  Created by Memo Akten on 14/08/2010.
 *  Copyright 2010 MSA Visuals Ltd. All rights reserved.
 *
 */

#include "MSACvStereoSolver.h"
#include "ofxCv/Utilities.h"

//template <class T>
void cvMatToTexture(cv::Mat& mat, ofTexture& tex) {
    // TODO: check allocation, don't allocate if not nessecary
    if(tex.getWidth() != mat.cols && tex.getHeight() != mat.rows) {
        tex.allocate(mat.cols, mat.rows, mat.channels() == 1 ? GL_LUMINANCE : GL_RGB);
    }
    tex.loadData(mat.ptr<unsigned char>(), mat.cols, mat.rows, mat.channels() == 1 ? GL_LUMINANCE : GL_RGB);
    //    img.getPixelsRef().setFromPixels(mat.ptr<unsigned char>(), mat.cols, mat.rows, );
    
}


namespace MSA {
    namespace Cv {
        
        //----------------------------------------
        StereoSolver::StereoSolver() {
            _isCalibrated           = false;
            //            _isAllocated            = true;
            _isCalibrating          = false;
            _assumeRectifiedInput   = false;
            
            settings.calibration.useCalibrated = true;
            settings.calibration.grabTime   = 5;
            settings.calibration.totalImageCount  = 12;
            settings.calibration.doDrawLines = true;
            
            settings.stereo.disparityMethod	= STEREO_BM;
            settings.stereo.depthMin = 0;
            settings.stereo.depthMax = 255;
            settings.stereo.depthNormalize = true;
        }
        
        
        //----------------------------------------
        //        StereoSolver::~StereoSolver() {
        //            clear();
        //        }	
        //        
        //        
        //        void StereoSolver::clear() {
        ////            if(_isAllocated) {
        ////            }
        //        }
        //
        //        
        //        //----------------------------------------
        //        void StereoSolver::allocate(int w, int h) {
        //            _imageSize = cv::Size(w, h);
        //            
        //        }
        
        
        //----------------------------------------
        void StereoSolver::startCalibrating(int numx, int numy, float squareSize) {
            //            for(int i=0; i<2; i++) {
            //                featureAccumulator[i].clear();
            //                chessboardFinder[i].allocate(numx, numy);
            //                chessboardFinder[i].settings.squareSize = squareSize;
            //            }
            
            imagePoints[0].resize(settings.calibration.totalImageCount);
            imagePoints[1].resize(settings.calibration.totalImageCount);
            objectPoints.clear();
            
            boardSize = cv::Size(numx, numy);
            settings.calibration.squareSize = squareSize;
            
            curGrabCount = 0;
            
            _isCalibrated = false;
            _isCalibrating = true;
            
            lastCalibrationGrabTime = ofGetElapsedTimef();
        }
        
        
        //----------------------------------------
        void StereoSolver::finishCalibrating() {
            _isCalibrating = false;
            
            imagePoints[0].resize(curGrabCount);
            imagePoints[1].resize(curGrabCount);
            objectPoints.resize(curGrabCount);
            
            for(int i = 0; i < curGrabCount; i++ ) {
                for(int j = 0; j < boardSize.height; j++ )
                    for(int k = 0; k < boardSize.width; k++ )
                        objectPoints[i].push_back(cv::Point3f(j*settings.calibration.squareSize, k*settings.calibration.squareSize, 0));
            }

            findCalibrationParams();
            findRectifyParams();
            calculateRectifyMaps();
            
        }
        
        
        //----------------------------------------
        void StereoSolver::saveCalibration(string filename) {
            cv::FileStorage fs(ofToDataPath(filename), CV_STORAGE_WRITE);
            if( fs.isOpened() )
            {
                fs << "M0" << _M[0] << "D0" << _D[0] << "M1" << _M[1] << "D1" << _D[1];
                fs << "R" << _R << "T" << _T  << "E" << _E << "F" << _F << "RR0" << _RR[0] << "RR1" << _RR[1] << "RP0" << _RP[0] << "RP1" << _RP[1] << "Q" << _Q;
                fs.release();
            }
            else
                cout << "Error: can not save the StereoCalibration parameters\n";
        }
        
        
        //----------------------------------------
        void StereoSolver::loadCalibration(string filename) {
            
        }
        
        
        //----------------------------------------
        bool StereoSolver::isCalibrating() {
            return _isCalibrating;
        }
        
        
        //----------------------------------------
        void StereoSolver::findFundamentalMatrix(vector<CvPoint2D32f>&p0, vector<CvPoint2D32f>&p1) {
            //            if(_isAllocated == false) {
            //                ofLog(OF_LOG_WARNING, "StereoSolver::findFundamentalMatrix - not allocated");
            //                return;
            //            }
            //            
            //            int pointCount = min(p0.size(), p1.size());
            //            _imagePoints[0] = cvMat(1, pointCount, CV_32FC2, &p0[0]);
            //            _imagePoints[1] = cvMat(1, pointCount, CV_32FC2, &p1[0]);
            //            
            //            bool found = cvFindFundamentalMat(&_imagePoints[0], &_imagePoints[1], _fundamentalMatrix, CV_FM_RANSAC, 1.0, 0.99);
            //            if(!found) {
            //                ofLog(OF_LOG_WARNING, "StereoSolver::findFundamentalMatrix not found");
            //            } else {
            //                ofLog(OF_LOG_VERBOSE, "StereoSolver::findFundamentalMatrix found");
            //            }
            //            
        }
        
        
        //----------------------------------------
        void StereoSolver::findCalibrationParams() {
            // CALIBRATE THE STEREO CAMERAS
            ofLog(OF_LOG_VERBOSE, "Running stereo calibration ...");
            
            for(int i=0; i<2; i++) {
                _M[i]   = cv::Mat::eye(3, 3, CV_64F);
            }
            
            // TODO: artifacts due to running calibration after every chessboard?
            
            double rms = cv::stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                             _M[0], _D[0],
                                             _M[1], _D[1],
                                             cv::Size(origMat[0].cols, origMat[0].rows), _R, _T, _E, _F,
                                             cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                             CV_CALIB_FIX_ASPECT_RATIO +
                                             CV_CALIB_ZERO_TANGENT_DIST +
                                             CV_CALIB_SAME_FOCAL_LENGTH +
                                             CV_CALIB_RATIONAL_MODEL +
                                             CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
            cout << "done with RMS error=" << rms << endl;
            
            //vector<CvPoint3D32f> lines[2];
            //	points[0].resize(N);
            //	points[1].resize(N);
            //	_imagePoints[0] = cvMat(1, N, CV_32FC2, &points[0][0] );
            //	_imagePoints[1] = cvMat(1, N, CV_32FC2, &points[1][0] );
            //	lines[0].resize(N);
            //	lines[1].resize(N);
            //	CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
            //	CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
            //	//Always work in undistorted space
            //	cvUndistortPoints( &_imagePoints[0], &_imagePoints[0], &_M[0], &_D[0], 0, &_M[0] );
            //	cvUndistortPoints( &_imagePoints[1], &_imagePoints[1], &_M[1], &_D[1], 0, &_M[1] );
            //	cvComputeCorrespondEpilines( &_imagePoints[0], 1, &_fundamentalMatrix, &_L1 );
            //	cvComputeCorrespondEpilines( &_imagePoints[1], 2, &_fundamentalMatrix, &_L2 );
            //	double avgErr = 0;
            //	for(int i = 0; i < N; i++ ) {
            //		double err = fabs(points[0][i].x*lines[1][i].x + points[0][i].y*lines[1][i].y + lines[1][i].z) + fabs(points[1][i].x*lines[0][i].x + points[1][i].y*lines[0][i].y + lines[0][i].z);
            //		avgErr += err;
            //	}
            //	printf( "avg err = %g\n", avgErr/N );
        }
        
        
        //----------------------------------------
        void StereoSolver::findRectifyParams() {
            //            if(_isAllocated == false) {
            //                ofLog(OF_LOG_WARNING, "StereoSolver::findRectifyParams - not allocated");
            //                return;
            //            }
            
            cv::Rect validRoi[2];
            cv::Size imageSize(origMat[0].cols, origMat[0].rows);
            cv::stereoRectify(_M[0], _D[0],
                              _M[1], _D[1],
                              imageSize, _R, _T, _RR[0], _RR[1], _RP[0], _RP[1], _Q,
                              cv::CALIB_ZERO_DISPARITY, -1, imageSize, &validRoi[0], &validRoi[1]);
            
            if(settings.calibration.useCalibrated) {
                // we already computed everything
            }
            // OR ELSE HARTLEY'S METHOD
            else
                // use intrinsic parameters of each camera, but
                // compute the rectification transformation directly
                // from the fundamental matrix
            {
                //                vector<cv::Point2f> allimgpt[2];
                //                for(int k=0; k<2; k++) {
                //                    for(int i=0; i<nimages; i++)
                //                        std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
                //                }
                //                F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
                //                Mat H1, H2;
                //                stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
                //                
                //                _RR[0] = cameraMatrix[0].inv()*H1*cameraMatrix[0];
                //                _RR[1] = cameraMatrix[1].inv()*H2*cameraMatrix[1];
                //                _RP[0] = cameraMatrix[0];
                //                _RP[1] = cameraMatrix[1];
            }
        }
        
        
        void StereoSolver::calculateRectifyMaps() {
            
            //Precompute maps for cv::remap()
            cv::initUndistortRectifyMap(_M[0], _D[0], _RR[0], _RP[0], cv::Size(origMat[0].cols, origMat[0].rows), CV_16SC2, _rmap[0][0], _rmap[0][1]);
            cv::initUndistortRectifyMap(_M[1], _D[1], _RR[1], _RP[1], cv::Size(origMat[0].cols, origMat[0].rows), CV_16SC2, _rmap[1][0], _rmap[1][1]);
            
            
            _isCalibrated = true;
        }
        
        
        void StereoSolver::assumeRectifiedInput(bool b) {
            _assumeRectifiedInput = b;
        }
        
        
        //----------------------------------------
        bool StereoSolver::rectifyImages(const cv::Mat& _img0, const cv::Mat& _img1, cv::Mat& _rimg0, cv::Mat& _rimg1) {
            
            // if input is already rectified, we just need to copy across
            if(_assumeRectifiedInput) {
                _rimg0 = _img0;
                _rimg1 = _img1;
                
            } else {   // otherwise we need to rectify
                
                // if we don't know the parameters, exit with warning
                if(_isCalibrated == false) {
                    ofLog(OF_LOG_WARNING, "StereoSolver::rectifyImages - not calibrated");
                    return false;
                }
                
                cv::remap(_img0, _rimg0, _rmap[0][0], _rmap[0][1], CV_INTER_LINEAR);
                cv::remap(_img1, _rimg1, _rmap[1][0], _rmap[1][1], CV_INTER_LINEAR);
            }
            
            return true;
        }
        
        
        //----------------------------------------
        bool StereoSolver::findDisparity(const cv::Mat& _rimg0, const cv::Mat& _rimg1, cv::Mat& _dimg) {

            switch(settings.stereo.disparityMethod) {
                case STEREO_BM:
                    bm(_rimg0, _rimg1, _dimg);
                    _dimg.convertTo(_dimg, CV_8U, 255/(bm.state->numberOfDisparities*16.0f));
                    break;
                    
                case STEREO_SGBM:
                    sgbm(_rimg0, _rimg1, _dimg);
                    _dimg.convertTo(_dimg, CV_8U, 255/(sgbm.numberOfDisparities*16.0f));
                    break;
            };
            
            //            cvNormalize( disp[0], _dispMat.getCvMat(), 0, 256, CV_MINMAX );   // TODO: make this an option
            
            
            return true;
        }
        
        
        //----------------------------------------
        bool StereoSolver::findDepthMap() {
            //            if(_isAllocated == false) {
            //                ofLog(OF_LOG_WARNING, "StereoSolver::findDepthMap - not allocated");
            //                return false;
            //            }
            
            //            cvReprojectImageTo3D(disp[0], depth3D, _reprojectionMatrix);
            //            
            //            cvSplit(depth3D, NULL, NULL, depth1D, NULL);
            //            
            //            if(settings.stereo.depthNormalize) {
            //                cvNormalize( depth1D, depthMat.getCvMat(), 0, 256, CV_MINMAX );
            //            } else {
            //                unsigned char *pixels = depthMat.getPixels();
            //                
            //                if(settings.stereo.depthMin==settings.stereo.depthMax) settings.stereo.depthMax++;
            //                
            //                for(int y=0; y<_imageSize.height; y++) {
            //                    float* fptr = (float*) (depth3D->imageData + y * depth3D->widthStep);
            //                    for(int x=0; x<_imageSize.width; x++) {
            //                        float fd = fptr[2];
            //                        //			if(ofRandomuf()<0.03) printf("%f ", fd); 
            //                        *pixels++ = ofMap(fd, settings.stereo.depthMin, settings.stereo.depthMax, 255, 0, true);
            //                        fptr+= 3;
            //                    }
            //                }
            //            }
            //	printf("\n");
            
            //	float* fptr= (float*)(depth->imageData); 
            //	float fv;
            //	double scaleit = 255.0 / (depthMax - depthMin);
            //
            //	for(int y=0; y<_imageSize.height; ++y) {
            //		for(int x=0; x<_imageSize.width; ++x) {
            //			fv = *(fptr+2);  // read the depth
            //			if((fv >= 0.0)||(fv > depthMax)) // too close
            //				*pixels++ = 0;
            //			else if(fv < depthMin)    // too far
            //				*pixels++ = 0;
            //			else 
            //				*pixels++ = (unsigned char)( scaleit*( (double)fv - depthMin) );      
            //		}
            //	}	
            //	
            
            //	depthMat.update();
            //            depthMat.flagImageChanged();
        }
        
        
        //----------------------------------------
        bool StereoSolver::update(ofImage& _img0, ofImage& _img1) {
            origImage[0] = &_img0;
            origImage[1] = &_img1;
            
            origMat[0] = ofxCv::toCv(_img0);
            origMat[1] = ofxCv::toCv(_img1);
            
            // if we are calibrating
            if(isCalibrating()) {
                // TODO: don't always do this, on timer? check motion?
                if(ofGetElapsedTimef() - lastCalibrationGrabTime > settings.calibration.grabTime) {
                    // look for chessboard in both images
                    int i = curGrabCount;
                    bool found[2];
                    for(int k=0; k<2; k++) {
                        vector<cv::Point2f>& corners = imagePoints[k][i];
                        found[k] = cv::findChessboardCorners(origMat[k], boardSize, corners, 
                                                             CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);                        
                        
                        if(found[k]) cv::cornerSubPix(origMat[k], corners, cv::Size(11,11), cv::Size(-1,-1),
                                                      cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                                                       30, 0.01));
                        
                    }
                    
                    if(found[0] && found[1]) {
                        curGrabCount++;
                        if(curGrabCount >= settings.calibration.totalImageCount) {
                            finishCalibrating();
                        }

                    } else {
                        ofLog(OF_LOG_VERBOSE, "StereoSolver::update - Number of features found in images do not match");
                    }
                    //                    
                    lastCalibrationGrabTime = ofGetElapsedTimef();
                }
            } else {
                if( rectifyImages(origMat[0], origMat[1], rectMat[0], rectMat[1]) == false) {
                    return false;
                }
                
                if( findDisparity(rectMat[0], rectMat[1], dispMat) == false) {
                    return false;
                }
                
                if( findDepthMap() == false) {
                    return false;
                }
            }
            //            dispMat.flagImageChanged();
            
            return true;
            
        }
        
        
        //----------------------------------------
        void StereoSolver::draw() {
            //            if(_isAllocated == false) {
            //                ofLog(OF_LOG_WARNING, "StereoSolver::draw - not allocated");
            //                return;
            //            }
            
            //            if(_isCalibrated == false) {
            //                ofLog(OF_LOG_WARNING, "StereoSolver::draw - not calibrated");
            //                return;
            //            }
            
            ofPushStyle();
            
            cv::Size imageSize(origMat[0].cols, origMat[0].rows);
            
            for(int k=0; k<2; k++) {
                float x = k * imageSize.width;
                ofSetColor(255, 255, 255);
                
                origImage[k]->draw(x, 0);
                
                cvMatToTexture(rectMat[k], rectImage[k]);
                rectImage[k].draw(x, imageSize.height);
                
                if(isCalibrating() && curGrabCount > 0) {
                    ofSetColor(255, 0, 0);
                    glPointSize(5);
                    vector<cv::Point2f>& corners = imagePoints[k][curGrabCount-1];

                    glBegin(GL_POINTS);
                    for(int i=0; i<corners.size(); i++) {
                        glVertex2f(corners[i].x+x, corners[i].y);
                    }
                    glEnd();

                }
            }
            
            if(isCalibrating()) {
                int y = imageSize.height + 20;
                ofSetColor(255, 0, 0);
                // ofDrawBitmapString("CALIBRATING - Looking for chessboard", 5, y);
                
//                int numImages = featureAccumulator[0].getNumImages();
                
                for(int i=0; i<settings.calibration.totalImageCount; i++) {
                    if(i<curGrabCount) ofFill();
                    else ofNoFill();
                    ofCircle(i*14+310, y-4, 6);
                }
                
                ofFill();
                ofRect(0, y+3, ofMap(ofGetElapsedTimef(), lastCalibrationGrabTime, lastCalibrationGrabTime + settings.calibration.grabTime, 0, imageSize.width*2), 5);
            }
            
            if(settings.calibration.doDrawLines) {
                ofColor colors[] = { ofColor(0, 0, 255), ofColor(0, 0, 0), ofColor(128, 128, 128), ofColor(255, 255, 255) };
                for(int i=0, c=0; i<imageSize.height*2; i+=10, c++) {
                    ofSetColor(colors[c%4]);
                    ofLine(0, i, imageSize.width*2, i);
                }
            }
            
            ofSetColor(255, 255, 255);
            cvMatToTexture(dispMat, dispImage);
            dispImage.draw(0, imageSize.height*2);
            
            //            depthMat.draw(_imageSize.width, _imageSize.height*2);
            
            
            ofPopStyle();
        }
        
        
        //----------------------------------------
        void StereoSolver::draw(float x, float y, float h) {
            glPushMatrix();
            glTranslatef(x, y, 0);
            
            float s = h/(3 * origMat[0].rows);
            glScalef(s, s, 1);
            
            draw();
            
            glPopMatrix();
        }
        
    }
}
