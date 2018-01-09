#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <sensor_msgs/image_encodings.h>
#include <detector_msgs/DetectorService.h>

#include <dlib/image_processing.h>

using namespace cv;

class FaceDetector {
public:
    FaceDetector()
    : detector(dlib::get_frontal_face_detector()) {
        service = n.advertiseService("detectFace", &FaceDetector::detectFace, this);

        dlib::deserialize("/home/alexandr/stereoDetector/src/real_face_detector/data/shape_predictor_68_face_landmarks.dat") >> sp;
    }

    ~FaceDetector() {

    }

    bool detectFace(detector_msgs::DetectorService::Request &req,
                 detector_msgs::DetectorService::Response &res) {

        cv_bridge::CvImagePtr cv_ptr_left;
        cv_bridge::CvImagePtr cv_ptr_right;
        try
        {
            cv_ptr_left = cv_bridge::toCvCopy(req.left_image, sensor_msgs::image_encodings::BGR8);
            cv_ptr_right = cv_bridge::toCvCopy(req.right_image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }

        Mat imgForDlib = cv_ptr_left->image.clone();

        dlib::cv_image<dlib::bgr_pixel> cimg(imgForDlib);
        faces_dlib = detector(cimg);

        
        faces.clear();
        for (unsigned i = 0; i != faces_dlib.size(); ++i) {
          //l = x
          //t = y
          //r = x+w
          //b = y+h

          Rect newFace((faces_dlib[i].left()), //x
                       (faces_dlib[i].top()), //y
                       (faces_dlib[i].right()-faces_dlib[i].left()), //width
                       (faces_dlib[i].bottom()-faces_dlib[i].top())  //height
                     ); // height

          faces.push_back(newFace);

        }
        
        
        //std::vector<dlib::full_object_detection> shapes;
        //shapes.push_back(shape);

        //win.clear_overlay();
        //win.set_image(cimg);
        //win.add_overlay(dlib::render_face_detections(shapes));

        //cv_ptr_left->image = drawFaces(imgForDlib, "face");
        //cv_ptr_left->image = imgForDlib(Rect(0,0,0,0));
        //cv_ptr_left->toImageMsg(res.face_image);

        if (faces.empty()) {
            res.rect.x = 0;
            res.rect.y = 0;
            res.rect.width = 0;
            res.rect.height = 0;

            res.face_rect.x = 0;
            res.face_rect.y = 0;
            res.face_rect.width = 0;
            res.face_rect.height = 0;
        } else {
            
            res.face_rect.x = faces.front().x;
            res.face_rect.y = faces.front().y;
            res.face_rect.width = faces.front().width;
            res.face_rect.height = faces.front().height;
            
            
            dlib::full_object_detection shape = sp(cimg, faces_dlib[0]);

            res.rect.x = shape.part(1).x();
            res.rect.y = shape.part(1).y();
            res.rect.width = shape.part(16).x() - shape.part(1).x();
            res.rect.height = shape.part(4).y() - shape.part(1).y();
            
            
        }
        return true;
    }

private:

    cv::Mat drawFaces(cv::Mat myImage, std::string str) {

        for (auto i = faces.begin(); i != faces.end(); ++i) {
            cv::rectangle(
              myImage,
              cv::Point((i->x), (i->y)),
              cv::Point((i->x) + (i->width), (i->y) + (i->height)),
              CV_RGB(50, 255 , 50),
              2);
            cv::putText(myImage, str, cv::Point((i->x), (i->y)), CV_FONT_NORMAL, 0.75, Scalar(255,50,50),1,1);
        }
        return myImage;
    }

    ros::NodeHandle n;
    ros::ServiceServer service;

    dlib::frontal_face_detector detector;
    dlib::shape_predictor sp;
    //dlib::image_window win;
    std::vector<cv::Rect> faces;
    std::vector<cv::Rect> faces_fromDlib;

    std::vector<dlib::rectangle> faces_dlib;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_detector");
    FaceDetector FaceDetector;
    ROS_INFO("Ready to add two ints.");
    ros::spin();

    return 0;
}
