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


#ifndef VIDEO_PLAYBACK_H
#define VIDEO_PLAYBACK_H

#include <iostream>
#include <string>
#include <memory>
#include <boost/shared_array.hpp>
#include <boost/thread.hpp>
#include <list>

namespace VideoHandler
{
/**
 * Decoding queue for a video stream.
 * It decodes on a worker thread the pushed video frames (jpeg) and renders them.
 * It uses a waiting queue (ring buffer) for caching.
 */
class VideoPlayback
{
public:

    /**
     * Container class to encapsulate an encoded jpeg video frame.
     */
    typedef struct EncodedVideoFrame {
        /** Array holding the video data. */
        boost::shared_array<char> data;

        /** The length of the data in bytes. */
        const unsigned int dataLength;

        /** Constructor. */
        EncodedVideoFrame() : data(), dataLength(0) { }

        /**
         * Constructor.
         * @param[in] _data Array holding the video data.
         * @param[in] _dataLength The length of the data in bytes.
         */
        EncodedVideoFrame(boost::shared_array<char> _data, const unsigned int _dataLength)
            : data(_data), dataLength(_dataLength) { }
    } EncodedVideoFrame;

    /**
     * Constructor.
     * @param[in] frameStreamId The ID of the framestream that provides the video frames.
     * @param[in] videoFramesMaxLength The size of the video frame waiting queue.
     */
    VideoPlayback(unsigned int frameStreamId, unsigned int videoFramesMaxLength);

    /** Destructor. */
    ~VideoPlayback();

    /**
     * Pushes a video frame into the waiting queue.
     * @param[in] data Array holding the video data.
     * @param[in] dataLength The length of the data in bytes.
     */
    void pushVideoFrame(boost::shared_array<char> data, const unsigned int dataLength);

private:

    /** The implementation of the decoding process run on the worker thread. */
    void decodeVideoFrame();

    /** The ID of the framestream that provides the video frames. */
    unsigned int m_frameStreamId;

    /** The label of the rendering window. */
    std::string m_windowName;

    /** The worker thread. */
    std::auto_ptr<boost::thread> m_videoPlaybackThread;

    /** Flag indicating the state of the worker thread. */
    bool m_threadRunning;

    /** The waiting queue for the video frames. */
    std::list<EncodedVideoFrame> m_videoFrames;

    /** The size of the video frame waiting queue. */
    unsigned int m_videoFramesMaxLength;

    /** Mutex for synchronization. */
    boost::mutex m_access;

    /** Condition variable to notify the worker thread when new video frame is available. */
    boost::condition_variable m_condition;
};

} // end namespace VideoHandler

#endif //VIDEO_PLAYBACK_H
