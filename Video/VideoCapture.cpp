/* Teleoperation System for Mobile Manipulators Framework
 *
 * Copyright (C) 2015 
 * RECONFIGURABLE CONTROL OF ROBOTIC SYSTEMS OVER NETWORKS Project
 * Robotics and Control Systems Laboratory
 * Department of Electrical Engineering
 * Sapientia Hungarian University of Transylvania
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * For more details see the project homepage:
 * <http://www.ms.sapientia.ro/~martonl/MartonL_Research_TE.htm>
 */


#include "VideoCapture.h"
#include <fstream>
#include <iostream>

using namespace std;

namespace VideoHandler
{

VideoCapture::VideoCapture()
    : m_capture(NULL)
{
    getCaptureResolution(m_width, m_height);
}

VideoCapturePtr getVideoCapture()
{
    return VideoCapturePtr(new VideoCapture());
}

bool VideoCapture::initCapture(int device, unsigned int fps)
{
    m_capture = new cv::VideoCapture(device);
    m_capture->set(CV_CAP_PROP_FPS, fps);
    if (!m_capture->isOpened()) {
        m_capture = NULL;
        return false;
    }
    return true;
}

void VideoCapture::releaseCapture()
{
    if (m_capture != NULL) {
        m_capture->release();
    }
}

void VideoCapture::setCaptureResolution(int width, int height)
{
    if (m_capture != NULL) {
        m_capture->set(CV_CAP_PROP_FRAME_WIDTH, width);
        m_capture->set(CV_CAP_PROP_FRAME_HEIGHT, height);
    }
}

void VideoCapture::getCaptureResolution(int& width, int& height)
{
    if (m_capture != NULL) {
        width = (int)m_capture->get(CV_CAP_PROP_FRAME_WIDTH);
        height = (int)m_capture->get(CV_CAP_PROP_FRAME_HEIGHT);
    }
    else {
        width = -1;
        height = -1;
    }
}

int VideoCapture::captureFrame(boost::shared_array<char>& data)
{
    if (m_capture != NULL) {
        cv::Mat frame;
        *m_capture >> frame;
        int size = 0;
        if (!frame.empty()) {
            size = frame.step * frame.rows;
            data = boost::shared_array<char>(new char[size]);
            memcpy(data.get(), frame.data, size);
        }
        return size;
    }
    return -1;
}


int VideoCapture::captureJPEG(boost::shared_array<char>& data, unsigned short compression, double x, double y, std::string textToDisplay)
{
    if (x < 0) x = 1;
    if (y < 0) y = 1;
    if (x > m_width) x = m_width;
    if (y > m_height) y = m_height;

    if (m_capture != NULL) {
        cv::Mat frame;
        *m_capture >> frame;

        if (!frame.empty()) {

            putText(frame, textToDisplay, cvPoint(5,20),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,0), 1, CV_AA);

            cv::circle(frame, cvPoint((int)x,(int)y), 10, cvScalar(0,0,255));
            cv::circle(frame, cvPoint((int)x,(int)y), 1, cvScalar(0,0,255), -1);

            std::vector<unsigned char> buf;
            std::vector<int> params;
            params.push_back(CV_IMWRITE_JPEG_QUALITY);
            params.push_back(compression);
            imencode(".jpg", frame, buf, params);

            int size = buf.size();
            data = boost::shared_array<char>(new char[size]);
            std::copy(buf.begin(), buf.end(), data.get());

            return size;
        } else
            cout << "VideoCapture: Empty frame was captured!" << endl;
    }
    return -1;
}

int VideoCapture::captureJPEG(boost::shared_array<char>& data, unsigned short compression)
{
    if (m_capture != NULL) {
        cv::Mat frame;
        *m_capture >> frame;

        if (!frame.empty()) {
            std::vector<unsigned char> buf;
            std::vector<int> params;
            params.push_back(CV_IMWRITE_JPEG_QUALITY);
            params.push_back(compression);
            imencode(".jpg", frame, buf, params);

            int size = buf.size();
            data = boost::shared_array<char>(new char[size]);
            std::copy(buf.begin(), buf.end(), data.get());

            return size;
        } else
            cout << "VideoCapture: Empty frame was captured!" << endl;
    }
    return -1;
}

} // end namespace VideoHandler

