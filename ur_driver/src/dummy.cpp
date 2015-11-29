// ----------------------------------------------------------------------------
// Copyright 2015 Fraunhofer IPA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
// This defines the communication between controller and ROS
// ----------------------------------------------------------------------------

#include <dummy.h>
#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>

using namespace std;
using namespace ur_driver;
using boost::asio::ip::tcp;

Dummy::Dummy() :
    runReadSocketThread(false),
    runWriteSocketThread(false),
    port(30002),
    isRunning(false)
{

}

Dummy::~Dummy()
{
    stop();
}

void Dummy::start(int port)
{
    mutexStartStop.lock();

    if (!isRunning)
    {
        ROS_DEBUG_NAMED("dummy", "start dummy server");

        this->port = port;

        acceptSocketThread = boost::thread(&Dummy::acceptSocketWorker, this, boost::ref(io));

        isRunning = true;
    }

    mutexStartStop.unlock();
}

void Dummy::stop()
{
    mutexStartStop.lock();

    if (isRunning)
    {
        ROS_DEBUG_NAMED("dummy", "stop dummy server");

        io.stop();

        runReadSocketThread = false;
        runWriteSocketThread = false;

        acceptSocketThread.join();
        readSocketThread.join();
        writeSocketThread.join();

        io.reset();

        isRunning = false;
    }

    mutexStartStop.unlock();
}

void Dummy::readSocketWorker(boost::shared_ptr<tcp::socket> socket)
{
    try
    {
        while(socket->is_open())
        {
            char data[1024];

            //read data from socket
            boost::system::error_code error;
            size_t length = socket->read_some(boost::asio::buffer(data), error);

            //connection closed cleanly by peer.
            if (error == boost::asio::error::eof)
            {
                socket->close();

                break;
            }
            //some other error
            else if (error)
            {
                throw boost::system::system_error(error);
            }
        }
    }
    catch (std::exception& e)
    {
        ROS_WARN("dummy: exception in read thread: %s", e.what());
    }

    ROS_DEBUG_NAMED("dummy", "dummy: exit readSocketWorker thread");
}

void Dummy::writeSocketWorker(boost::shared_ptr<tcp::socket> socket)
{
    try
    {
        ros::Rate loopRate(10);

        while (socket->is_open())
        {
            //build dummy data
            std::stringstream stream;
            stream <<(char)0 <<(char)0 <<(char)1 <<(char)209 <<(char)16 <<(char)0 <<(char)0 <<(char)0 <<(char)29 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)5 <<(char)7 <<(char)246 <<(char)1 <<(char)1 <<(char)1 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)63 <<(char)240 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0;
            stream <<(char)0 <<(char)251 <<(char)1 <<(char)64 <<(char)1 <<(char)78 <<(char)244 <<(char)77 <<(char)189 <<(char)249 <<(char)149 <<(char)64 <<(char)1 <<(char)78 <<(char)247 <<(char)89 <<(char)95 <<(char)104 <<(char)85 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)188 <<(char)220 <<(char)97 <<(char)3 <<(char)66 <<(char)62;
            stream <<(char)0 <<(char)0 <<(char)66 <<(char)0 <<(char)102 <<(char)103 <<(char)66 <<(char)99 <<(char)153 <<(char)154 <<(char)253 <<(char)191 <<(char)246 <<(char)74 <<(char)170 <<(char)216 <<(char)242 <<(char)29 <<(char)102 <<(char)191 <<(char)246 <<(char)74 <<(char)178 <<(char)44 <<(char)92 <<(char)72 <<(char)137 <<(char)0 <<(char)0 <<(char)0 <<(char)0;
            stream <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)192 <<(char)1 <<(char)106 <<(char)78 <<(char)66 <<(char)63 <<(char)153 <<(char)154 <<(char)66 <<(char)4 <<(char)204 <<(char)205 <<(char)66 <<(char)104 <<(char)0 <<(char)0 <<(char)253 <<(char)63 <<(char)253 <<(char)49 <<(char)202 <<(char)91 <<(char)202 <<(char)8 <<(char)64 <<(char)63 <<(char)253 <<(char)49;
            stream <<(char)210 <<(char)233 <<(char)51 <<(char)16 <<(char)35 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)191 <<(char)155 <<(char)135 <<(char)34 <<(char)66 <<(char)62 <<(char)0 <<(char)0 <<(char)65 <<(char)249 <<(char)153 <<(char)154 <<(char)66 <<(char)100 <<(char)0 <<(char)0 <<(char)253 <<(char)191 <<(char)220 <<(char)115;
            stream <<(char)204 <<(char)104 <<(char)205 <<(char)239 <<(char)254 <<(char)191 <<(char)220 <<(char)118 <<(char)68 <<(char)109 <<(char)49 <<(char)34 <<(char)158 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)190 <<(char)60 <<(char)245 <<(char)109 <<(char)66 <<(char)63 <<(char)153 <<(char)154 <<(char)66 <<(char)25 <<(char)51;
            stream <<(char)51 <<(char)66 <<(char)116 <<(char)204 <<(char)205 <<(char)253 <<(char)63 <<(char)242 <<(char)146 <<(char)224 <<(char)105 <<(char)231 <<(char)66 <<(char)209 <<(char)63 <<(char)242 <<(char)146 <<(char)65 <<(char)3 <<(char)193 <<(char)196 <<(char)156 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)190 <<(char)115;
            stream <<(char)157 <<(char)190 <<(char)66 <<(char)62 <<(char)0 <<(char)0 <<(char)66 <<(char)21 <<(char)51 <<(char)51 <<(char)66 <<(char)119 <<(char)153 <<(char)154 <<(char)253 <<(char)191 <<(char)231 <<(char)207 <<(char)8 <<(char)215 <<(char)85 <<(char)22 <<(char)88 <<(char)191 <<(char)231 <<(char)206 <<(char)77 <<(char)130 <<(char)151 <<(char)190 <<(char)17;
            stream <<(char)191 <<(char)146 <<(char)242 <<(char)158 <<(char)148 <<(char)114 <<(char)240 <<(char)57 <<(char)188 <<(char)224 <<(char)224 <<(char)96 <<(char)66 <<(char)68 <<(char)102 <<(char)103 <<(char)66 <<(char)37 <<(char)153 <<(char)154 <<(char)66 <<(char)127 <<(char)153 <<(char)154 <<(char)253 <<(char)0 <<(char)0 <<(char)0 <<(char)53 <<(char)4 <<(char)63;
            stream <<(char)217 <<(char)153 <<(char)52 <<(char)224 <<(char)36 <<(char)238 <<(char)93 <<(char)191 <<(char)217 <<(char)153 <<(char)169 <<(char)67 <<(char)241 <<(char)211 <<(char)23 <<(char)63 <<(char)207 <<(char)255 <<(char)137 <<(char)8 <<(char)22 <<(char)253 <<(char)198 <<(char)63 <<(char)240 <<(char)0 <<(char)170 <<(char)111 <<(char)207 <<(char)54;
            stream <<(char)176 <<(char)63 <<(char)243 <<(char)51 <<(char)88 <<(char)137 <<(char)58 <<(char)151 <<(char)217 <<(char)63 <<(char)201 <<(char)148 <<(char)119 <<(char)151 <<(char)70 <<(char)237 <<(char)237 <<(char)0 <<(char)0 <<(char)0 <<(char)29 <<(char)5 <<(char)64 <<(char)143 <<(char)64 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)64 <<(char)143 <<(char)64;
            stream <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)64 <<(char)143 <<(char)64 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)61 <<(char)3 <<(char)0 <<(char)63 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)63 <<(char)123 <<(char)129 <<(char)184 <<(char)27 <<(char)129 <<(char)184 <<(char)28 <<(char)63 <<(char)112 <<(char)225;
            stream <<(char)14 <<(char)16 <<(char)225 <<(char)14 <<(char)17 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)0 <<(char)66 <<(char)87 <<(char)51 <<(char)51 <<(char)66 <<(char)66 <<(char)0 <<(char)0 <<(char)62 <<(char)35 <<(char)215 <<(char)11 <<(char)61;
            stream <<(char)241 <<(char)169 <<(char)253 <<(char)0 <<(char)0 <<(char)0 <<(char)37 <<(char)2 <<(char)0 <<(char)0 <<(char)63 <<(char)141 <<(char)83 <<(char)47 <<(char)180 <<(char)171 <<(char)196 <<(char)232 <<(char)63 <<(char)137 <<(char)46 <<(char)99 <<(char)102 <<(char)69 <<(char)149 <<(char)155 <<(char)66 <<(char)55 <<(char)51 <<(char)51 <<(char)0 <<(char)59;
            stream <<(char)68 <<(char)155 <<(char)166 <<(char)66 <<(char)92 <<(char)0 <<(char)0 <<(char)253;

            const char* data = stream.str().c_str();
            int length = stream.str().length();

            //write data to socket
            boost::system::error_code error;
            socket->write_some(boost::asio::buffer(data, length), error);

            //connection closed cleanly by peer.
            if (error == boost::asio::error::eof)
            {
                break;
            }
            //some other error
            else if (error)
            {
                throw boost::system::system_error(error);
            }

            loopRate.sleep();
        }
    }
    catch (std::exception& e)
    {
        ROS_WARN("dummy: exception in write thread: %s", e.what());
    }

    ROS_DEBUG_NAMED("dummy", "dummy: exit writeSocketWorker thread");
}

void Dummy::acceptSocketWorker(boost::asio::io_service& io)
{
    try
    {
        tcp::acceptor acceptor(io, tcp::endpoint(tcp::v4(), port));

        boost::shared_ptr<tcp::socket> socket(new tcp::socket(acceptor.get_io_service()));
        acceptor.async_accept(*socket, boost::bind(&Dummy::handleAccept, this, boost::asio::placeholders::error, socket, boost::ref(acceptor)));

        io.run();

        readSocketThread.join();
        writeSocketThread.join();
    }
    catch (std::exception& e)
    {
        ROS_ERROR("dummy: server error: %s", e.what());
    }

    ROS_DEBUG_NAMED("dummy", "dummy: exit acceptSocketWorker thread");
}

void Dummy::handleAccept(const boost::system::error_code& error, boost::shared_ptr<boost::asio::ip::tcp::socket> socket, boost::asio::ip::tcp::acceptor& acceptor)
{
    if (error)
    {
        return;
    }

    ROS_DEBUG_NAMED("dummy", "dummy: accepted client");

    runReadSocketThread = true;
    runWriteSocketThread = true;
    readSocketThread = boost::thread(boost::bind(&Dummy::readSocketWorker, this, socket));
    writeSocketThread = boost::thread(boost::bind(&Dummy::writeSocketWorker, this, socket));
}
