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


#ifndef VIDEO_CAPTURE_H
#define VIDEO_CAPTURE_H

#include "IVideoCapture.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types_c.h>

namespace VideoHandler
{
/**
 * Video capture implementation.
 */
class VideoCapture : public IVideoCapture
{
public:

    /**
     * Constructor.
     */
    VideoCapture();


    /**
     * Initiates the capturing.
     * @param[in] device The camera ID.
     * @param[in] fps The required frame rate.
     * @return True if the capturing was successfully initialized, otherwise false.
     */
    virtual bool initCapture(int device, unsigned int fps);

    /**
     * Releases the camera and stops the capturing.
     */
    virtual void releaseCapture();

    /**
     * Sets the capturing resolution.
     * @param[in] width Frame width.
     * @param[in] height Frame height.
     */
    virtual void setCaptureResolution(int width, int height);

    /**
     * Getter for the capturing resolution.
     * @param[out] width Frame width.
     * @param[out] height Frame height.
     */
    virtual void getCaptureResolution(int& width, int& height);

    /**
     * Captures a raw frame.
     * @param[out] data Array with the video frame data.
     * @return The size of the captured frame in bytes or negative if no frame was captured.
     */
    virtual int captureFrame(boost::shared_array<char>& data);

    /**
     * Captures a jpeg frame.
     * @param[out] data Array with the video frame data.
     * @param[in] compression The desired jpeg compression rate.
     * @return The size of the captured frame in bytes or negative if no frame was captured.
     */
    virtual int captureJPEG(boost::shared_array<char>& data, unsigned short compression);

    /**
     * Captures a jpeg frame and draws a pointer overlay.
     * @param[out] data Array with the video frame data.
     * @param[in] compression The desired jpeg compression rate.
     * @param[in] x The center of the pointer on axes X.
     * @param[in] y The center of the pointer on axes Y.
     * @param[in] textToDisplay The given text is displayed on the top left corner of the image.
     * @return The size of the captured frame in bytes or negative if no frame was captured.
     */
    virtual	int captureJPEG(boost::shared_array<char>& data, unsigned short compression, double x, double y, std::string textToDisplay);

private:
    /** Pointer to capturer. */
    cv::VideoCapture* m_capture;

    /** Video frame width. */
    int m_width;

    /** Video frame height. */
    int m_height;
};

} // end namespace VideoHandler

#endif //VIDEO_CAPTURE_H
