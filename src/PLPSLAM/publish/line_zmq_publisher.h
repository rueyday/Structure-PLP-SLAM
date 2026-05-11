/**
 * This file is part of Structure PLP-SLAM.
 * ZMQ-based real-time publisher for 3D line landmarks.
 */

#ifndef PLPSLAM_PUBLISH_LINE_ZMQ_PUBLISHER_H
#define PLPSLAM_PUBLISH_LINE_ZMQ_PUBLISHER_H

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <zmq.hpp>

namespace PLPSLAM
{
    namespace publish
    {
        class map_publisher;
    }

    namespace publish
    {
        /**
         * Publishes all valid 3D line landmarks via a ZMQ PUB socket at a
         * configurable interval.  Each message is a MessagePack-encoded array:
         *   [ {"id": uint, "sp": [x,y,z], "ep": [x,y,z], "num_obs": uint}, ... ]
         *
         * Python consumer:
         *   import zmq, msgpack
         *   s = ctx.socket(zmq.SUB); s.connect("tcp://localhost:5557")
         *   s.setsockopt(zmq.SUBSCRIBE, b"")
         *   lines = msgpack.unpackb(s.recv())
         */
        class line_zmq_publisher
        {
        public:
            /**
             * @param map_pub      shared map publisher (source of line data)
             * @param endpoint     ZMQ bind address, e.g. "tcp://*:5557" (all interfaces)
             * @param interval_sec publish interval in seconds (default 5.0)
             */
            line_zmq_publisher(map_publisher *map_pub,
                               const std::string &endpoint = "tcp://*:5557",
                               double interval_sec = 5.0);

            ~line_zmq_publisher();

            void run();
            void request_terminate();
            bool is_terminated() const;

        private:
            void publish_once();

            map_publisher *map_pub_;
            std::string endpoint_;
            std::chrono::milliseconds interval_;

            zmq::context_t ctx_;
            zmq::socket_t sock_;

            std::atomic<bool> terminate_requested_{false};
            std::atomic<bool> terminated_{false};
        };

    } // namespace publish
} // namespace PLPSLAM

#endif // PLPSLAM_PUBLISH_LINE_ZMQ_PUBLISHER_H
