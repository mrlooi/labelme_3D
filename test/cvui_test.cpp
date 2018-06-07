#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#define CVUI_IMPLEMENTATION

#include "cvui.h"
 
#define WINDOW_NAME "CVUI Hello World!"
 
typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayXb;


int main(void)
{
    cv::Mat lena = cv::imread("../data/sss.png");
    cv::Mat frame = lena.clone();
    int low_threshold = 50, high_threshold = 150;
    
    cv::namedWindow(WINDOW_NAME);
    cvui::init(WINDOW_NAME);

    ArrayXb modes = ArrayXb::Constant(3,false);

    bool& ANNOTATE = modes(0);
    bool& DELETE = modes(1);
    bool& MERGE = modes(2);

    ANNOTATE = true;

    std::vector<cv::Point> points;
    
    while (true) 
    {
 
        bool a_mode = ANNOTATE;
        bool d_mode = DELETE;
        bool m_mode = MERGE;

        bool undo_flag = false, redo_flag = false;

        cvui::window(frame, 10, 50, 180, 180, "Mode");
        cvui::checkbox(frame, 15, 80, "Annotate", &a_mode);
        cvui::checkbox(frame, 15, 120, "Delete", &d_mode);
        cvui::checkbox(frame, 15, 160, "Merge", &m_mode);

        cvui::window(frame, 10, 200, 180, 180, "Revert");
        cvui::checkbox(frame, 15, 240, "Undo", &undo_flag);
        cvui::checkbox(frame, 80, 240, "Redo", &redo_flag);

        cvui::printf(frame, 15, 280, 0.4, 0xff0000, "Button click count: %d", 3);

        cvui::update();
        cv::imshow(WINDOW_NAME, frame);
 
        if (cv::waitKey(30) == 27) {
            break;
        }

        if (undo_flag)
        {
            printf("UNDO!\n");
            continue;
        } else if (redo_flag)
        {
            printf("REDO!\n");
            continue;
        } else if (a_mode && !ANNOTATE)
        {
            modes = ArrayXb::Constant(3,false);
            ANNOTATE = true;
        }
        else if (d_mode && !DELETE)
        {
            modes = ArrayXb::Constant(3,false);
            DELETE = true;
        }
        else if (m_mode && !MERGE)
        {
            modes = ArrayXb::Constant(3,false);
            MERGE = true;
        }

        if (ANNOTATE)
        {
            if (cvui::mouse(cvui::DOWN)) {
                // Position the rectangle at the mouse pointer.
                cv::Point pt;
                pt.x = cvui::mouse().x;
                pt.y = cvui::mouse().y;
                cv::circle( frame, pt, 3, cv::Scalar(0,0,255), 3, 8, 0 );
            }
        } else if (DELETE) {

        } else if (MERGE) {

        }
        // cvui::trackbar(frame, 15, 110, 165, &low_threshold, 5, 150);
        // cvui::trackbar(frame, 15, 180, 165, &high_threshold, 80, 300);
 
    }
    return 0;
}

