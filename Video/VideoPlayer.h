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


#ifndef VIDEO_PLAYER_H
#define VIDEO_PLAYER_H

#include <IVideoPlayer.h>
#include <VideoPlayback.h>

#include <queue>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types_c.h>

namespace VideoHandler
{
/** Video player implementation */
class VideoPlayer : public IVideoPlayer
{
public:
    /**
     * Constructor.
     */
    VideoPlayer();

    /**
     * Destructor.
     */
    ~VideoPlayer();

    /**
     * Pushes an encoded frame for decoding and playback.
     * @param[in] frameStreamId The unique ID of the video stream.
     * @param[in] data The binary data buffer.
     * @param[in] dataLength The length of the data in the buffer.
     */
    virtual void pushEncodedFrame(unsigned int frameStreamId, boost::shared_array<char> data, const unsigned int dataLength);

    /**
     * Adds a new video playback channel to the player. It has to be called once per video stream before pushing frames into the player.
     * @param[in] frameStreamId The unique ID of the video stream.
     * @param[in] maxDecodingFrameCount The size of the frame queue for the new video stream.
     */
    virtual void addVideoPlayback(unsigned int frameStreamId, unsigned int maxDecodingFrameCount);

private:
    /** Map to hold the video playback instances. The key is the stream ID and the value is a pointer to the video playback instance.  */
    std::map<unsigned int, boost::shared_ptr<VideoPlayback> > m_videoPlaybacks;
    /** Mutex used for synchronization */
    boost::mutex m_access;
};

} // end namespace VideoHandler

#endif //VIDEO_PLAYER_H
