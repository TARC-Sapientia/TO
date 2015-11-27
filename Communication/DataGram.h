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


#pragma once
#ifndef DataGram_h
#define DataGram_h

#include "DataGramPack.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>

using namespace std;
using boost::asio::ip::udp;

namespace Communication
{
  class FrameStream;

  /**
   * The class that houses the UDP socket connection used to send and receive datagrams asynchronously.
   * Incoming datagrams are serialized using a strand.
   */
  class DataGram
  {
    public:
      /**
       * Constructor. It immediately starts listening for incoming datagrams.
       * @param[in] frameStream The frame stream it belongs to.
       * @param[in] receiveBufferSize The size of the receive buffer of the socket.
       */
      DataGram(FrameStream& frameStream, unsigned int receiveBufferSize);

      /** Destructor. */
      ~DataGram();

      /**
       * Used to send a datagram (header + payload) asynchronously using the socket connection.
       * The function call always returns immediately.
       * OnAsyncSend is called when the operation completes.
       * @param[in] dataGramPackPtr The datagram to be sent.
       */
      void AsyncSend(boost::shared_ptr<DataGramPack> dataGramPackPtr);

      /**
       * Handler called when an asynchronous send completes.
       * @param[in] error_code The error's code, if any.
       * @param[in] bytesSent The number of bytes sent successfully.
       * @param[in] message The datagram that was sent. It will be deallocated now.
       */
      void OnAsyncSend(const boost::system::error_code& error_code, size_t bytesSent, char* message);
      
    protected:
      /**
       * Used to asynchronously receive a datagram from the remote endpoint.
       * The function call always returns immediately.
       * OnAsyncReceive is called when a datagram is received.
       */
      void AsyncReceive();

      /**
       * Handler called when an asynchronous read completes.
       * Calls to this function are serialized using a strand.
       * @param[in] error_code The error's code, if any.
       * @param[in] bytesReceived The number of bytes received.
       */
      void OnAsyncReceive(const boost::system::error_code& error_code, size_t bytesReceived);

      /** The buffer used to hold incoming datagrams. */
      boost::array<char, MAXIMUM_DATAGRAM_LENGTH> m_receiveBuffer;
      
      /** The size of the receive buffer. */
      size_t m_receiveBufferSize;
      
      /** The frame stream associated with the datagram. */
      FrameStream& m_FrameStream;
      
      /** The UDP socket used for communication. */
      udp::socket m_Socket;
      
    private:
      /** Default constructor. Not allowed. */
      DataGram();
      
      /** The strand used to serialize calls to the OnAsyncReceive handler function. */
      boost::asio::strand m_strand;
  };
  
} // end namespace Communication

#endif // DataGram_h
