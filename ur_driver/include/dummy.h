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

#ifndef DUMMY_H_
#define DUMMY_H_

#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

namespace ur_driver
{
    class Dummy {
        public:
            Dummy();
            ~Dummy();

            void start(int port);
            void stop();

        private:
            bool runReadSocketThread;
            bool runWriteSocketThread;
            boost::thread acceptSocketThread;
            boost::thread readSocketThread;
            boost::thread writeSocketThread;

            boost::asio::io_service io;

            int port;
            bool isRunning;

            boost::mutex mutexStartStop;

            void readSocketWorker(boost::shared_ptr<boost::asio::ip::tcp::socket> socket);
            void writeSocketWorker(boost::shared_ptr<boost::asio::ip::tcp::socket> socket);

            void acceptSocketWorker(boost::asio::io_service& io);
            void handleAccept(const boost::system::error_code& error, boost::shared_ptr<boost::asio::ip::tcp::socket> socket, boost::asio::ip::tcp::acceptor& acceptor);
    };
}

#endif
