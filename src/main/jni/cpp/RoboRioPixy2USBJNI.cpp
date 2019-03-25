//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
#include <jni.h>
#include <iostream>
#include <sstream>
#include <time.h>
#include <unistd.h>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "frc_robot_vision_Pixy2USBOpenCVCalibrate.h"
#include "libpixyusb2.h"

Pixy2 pixy;
uint8_t *bayerFrame;
cv::Mat bayerMat(PIXY2_RAW_FRAME_HEIGHT, PIXY2_RAW_FRAME_WIDTH, CV_8U);
cv::Mat output(PIXY2_RAW_FRAME_HEIGHT, PIXY2_RAW_FRAME_WIDTH, CV_8UC3);

std::vector<cv::Point3f> createChessboardObjectPoints()
{
   std::vector<cv::Point3f> return_vec;
   for (int i=0; i<7; ++i) {
      for (int j=0; j<6; ++j) {
         return_vec.push_back(cv::Point3f(i, j, 0));
      }
   }
   return return_vec;
}

JNIEXPORT jint JNICALL Java_frc_robot_vision_Pixy2USBOpenCVCalibrate_pixy2USBInit(JNIEnv *env, jobject thisObj) {
   std::cout << "INFO: Pixy2 USB init" << std::endl;
   return pixy.init();
}

JNIEXPORT void JNICALL Java_frc_robot_vision_Pixy2USBOpenCVCalibrate_pixy2USBGetVersion(JNIEnv *env, jobject thisObj) {
   std::cout << "INFO: Pixy2 USB get version" << std::endl;
   pixy.version->print();
   return;
}

JNIEXPORT void JNICALL Java_frc_robot_vision_Pixy2USBOpenCVCalibrate_pixy2USBLampOn(JNIEnv *env, jobject thisObj) {
   std::cout << "INFO: Pixy2 USB Lamp On" << std::endl;
   pixy.setLamp(0x01, 0x00);
   return;
}

JNIEXPORT void JNICALL Java_frc_robot_vision_Pixy2USBOpenCVCalibrate_pixy2USBLampOff(JNIEnv *env, jobject thisObj) {
   std::cout << "INFO: Pixy2 USB Lamp Off" << std::endl;
   pixy.setLamp(0x00, 0x00);
   return;
}

JNIEXPORT jstring JNICALL Java_frc_robot_vision_Pixy2USBOpenCVCalibrate_pixy2USBGetBlocks(JNIEnv *env, jobject thisObj)
{
  int  Block_Index;

  // Query Pixy for blocks //
  pixy.ccc.getBlocks();

  std::stringstream ss;

  // Were blocks detected? //
  if (pixy.ccc.numBlocks)
  {
    // Blocks detected - print them! //

    // Uncomment for debug
   //  printf ("Detected %d block(s)\n", pixy.ccc.numBlocks);

    for (Block_Index = 0; Block_Index < pixy.ccc.numBlocks; ++Block_Index)
    {
      // printf ("  Block %d: ", Block_Index + 1);
      ss << "block " << Block_Index + 1 << " : ";
      ss << pixy.ccc.blocks[Block_Index].str();
      if (Block_Index < pixy.ccc.numBlocks-1) {
          ss << std::endl;
      }
    //   pixy.ccc.blocks[Block_Index].print();
    }
  }
  return env->NewStringUTF(ss.str().c_str());
}

JNIEXPORT void JNICALL Java_frc_robot_vision_Pixy2USBOpenCVCalibrate_pixy2USBTakePicture(JNIEnv *env, jobject thisObj)
{
   std::cout << "INFO: Taking picture" << std::endl;
   // need to call stop() before calling getRawFrame().
   // Note, you can call getRawFrame multiple times after calling stop().
   // That is, you don't need to call stop() each time.
   pixy.m_link.stop();

   // grab raw frame, BGGR Bayer format, 1 byte per pixel
   pixy.m_link.getRawFrame(&bayerFrame);
   // convert Bayer frame to RGB frame
   bayerMat.data = bayerFrame;

   // Using OpenCV for conversion to RGB
   cv::cvtColor(bayerMat, output, cv::COLOR_BayerBG2RGB);

   // Call resume() to resume the current program, otherwise Pixy will be left in "paused" state.
   pixy.m_link.resume();

   time_t rawtime;
   struct tm * timeinfo;
   char output_filename [256];

   time (&rawtime);
   timeinfo = localtime (&rawtime);

   strftime (output_filename, 256, "/home/lvuser/images/checker_board_%m-%d-%Y-%H-%M-%S.jpg", timeinfo);
   puts(output_filename);

   cv::imwrite(output_filename, output);

   std::cout << "INFO: Picture taken" << std::endl;
}

JNIEXPORT void JNICALL Java_frc_robot_vision_Pixy2USBOpenCVCalibrate_pixy2Calibrate(JNIEnv *env, jobject thisObj)
{
   DIR *dir;
   struct dirent *ent;
   
   std::vector<std::string> filenames;
   // std::string prefix("checker_board");
   std::string prefix("left");
   bool retval;

   std::vector<cv::Point3f> objBasePoints = createChessboardObjectPoints();
   std::vector<std::vector<cv::Point3f> > objPoints;
   std::vector<std::vector<cv::Point2f> > imgPoints;
   std::vector<cv::Point2f> corners;
   cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
   cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
   cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
   std::vector<cv::Mat> rvecs;
   std::vector<cv::Mat> tvecs;


   if ((dir = opendir("/home/lvuser/images")) != NULL) {
      /* finds all the files and directories within directory */
      while ((ent = readdir(dir)) != NULL) {
         /* only keep file names with given prefix */
         std::string curfilename(ent->d_name);
         if (!curfilename.compare(0, prefix.size(), prefix)) {
            filenames.push_back(curfilename);
         }
      }
      closedir(dir);
   } else {
      /* could not open directory */
      perror ("");
   }

   cv::Size imgShape;

   for (std::string& filename : filenames) {
      std::string fullfilename = "/home/lvuser/images/" + filename;
      std::cout << "INFO: image filename: " << fullfilename << std::endl;
      cv::Mat img = cv::imread(fullfilename);
      cv::Mat gray;

      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

      if (imgShape != gray.size()) {
         imgShape = gray.size();
      }

      retval = cv::findChessboardCorners(img, cv::Size(7, 6), corners);

      if (retval) {
         std::cout << "INFO: Successfully found corners: "<< std::endl;

         objPoints.push_back(objBasePoints);

         cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

         imgPoints.push_back(corners);

      } else {
         std::cout << "INFO: Could not find corners: "<< std::endl;
      }
   }

   std::cout << "INFO: Start Calibration" << std::endl;

   cv::calibrateCamera(objPoints, imgPoints, imgShape, cameraMatrix, distCoeffs, rvecs, tvecs);

   std::cout << "INFO: cameraMatrix: " << cameraMatrix << std::endl;
   std::cout << "INFO: distCoeffs: " << distCoeffs << std::endl;
   std::cout << "INFO: rvec size: " << rvecs.size() << std::endl;

   for (cv::Mat& rvec : rvecs) {
      std::cout << rvec << std::endl;
   }
   
   std::cout << "INFO: tvec size: " << tvecs.size() << std::endl;

   std::cout << "INFO: End Calibration" << std::endl;
}
