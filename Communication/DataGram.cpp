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


#include <iostream>
#include "DataGram.h"
#include "FrameStream.h"
#include "Core.h"

using namespace Utils;

namespace Communication
{

  DataGram::DataGram(FrameStream& frameStream, unsigned int receiveBufferSize)
    : m_FrameStream(frameStream),
      m_Socket(Core::get().getIoService(),
               frameStream.GetLocalEndPoint()),
      m_strand(Core::get().getIoService())
  {
    boost::asio::socket_base::receive_buffer_size option(receiveBufferSize);
    m_Socket.set_option(option);
    boost::asio::socket_base::send_buffer_size options(8*1024);
    m_Socket.set_option(options);

    m_receiveBufferSize = receiveBufferSize;

    // Start receiving immediately
    AsyncReceive();
  }


  void DataGram::AsyncReceive()
  {
    // Receive the message, after that call OnAsyncReceive
    m_Socket.async_receive_from(
          boost::asio::buffer(m_receiveBuffer, m_receiveBufferSize),
          m_FrameStream.GetRemoteEndPoint(),
          m_strand.wrap(
            boost::bind(&DataGram::OnAsyncReceive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred)));
  }


  void DataGram::OnAsyncReceive(const boost::system::error_code& error_code, size_t bytesReceived)
  {
    if (!error_code || error_code == boost::asio::error::message_size)
    {
      // Allocate space
      boost::shared_ptr<DataGramPack> dataGramPackPtr = boost::shared_ptr<DataGramPack>(new DataGramPack());

      memcpy(dataGramPackPtr.get(), m_receiveBuffer.data(), sizeof(DataGramPackHeader));

      dataGramPackPtr->data = boost::shared_array<char>(new char[dataGramPackPtr->header.dataLength]);
      memcpy(dataGramPackPtr->data.get(), m_receiveBuffer.data() + sizeof(DataGramPackHeader), dataGramPackPtr->header.dataLength);

      m_FrameStream.OnRead(dataGramPackPtr);

      AsyncReceive();
    }
    else
    {
      cout << "Communication: receive error code: " << error_code << endl;
    }
  }


  void DataGram::AsyncSend(boost::shared_ptr<DataGramPack> dataGramPackPtr)
  {
    // Try to send message.
    // Call OnAsyncSend when finished

    uint32_t length = sizeof(DataGramPackHeader) + dataGramPackPtr->header.dataLength;
    char *message = new char[length];

    memcpy(message, dataGramPackPtr.get(), sizeof(DataGramPackHeader));
    memcpy(message + sizeof(DataGramPackHeader), dataGramPackPtr->data.get(), dataGramPackPtr->header.dataLength);

    m_Socket.async_send_to(
          boost::asio::buffer(message, length),
          m_FrameStream.GetRemoteEndPoint(),
          boost::bind(&DataGram::OnAsyncSend, this,
                      boost::asio::placeholders::error,
                      boost::asio::placeholders::bytes_transferred,
                      message));
  }


  void DataGram::OnAsyncSend(const boost::system::error_code& error_code, size_t bytesSent, char* message)
  {
    delete[] message;

    if (!error_code)
    {
      // notify user
      m_FrameStream.OnWrite();
    }
    else
      cout << "Communication: error send: " << error_code << endl;
  }


  DataGram::~DataGram()
  {
    m_Socket.close();
  }

} // end namespace Communication

