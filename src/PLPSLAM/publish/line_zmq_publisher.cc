/**
 * This file is part of Structure PLP-SLAM.
 * ZMQ-based real-time publisher for 3D line landmarks.
 */

#include "PLPSLAM/publish/line_zmq_publisher.h"
#include "PLPSLAM/publish/map_publisher.h"
#include "PLPSLAM/data/landmark_line.h"
#include "PLPSLAM/data/keyframe.h"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>

namespace PLPSLAM
{
    namespace publish
    {

        line_zmq_publisher::line_zmq_publisher(map_publisher *map_pub,
                                               const std::string &endpoint,
                                               double interval_sec)
            : map_pub_(map_pub),
              endpoint_(endpoint),
              interval_(static_cast<long long>(interval_sec * 1000)),
              ctx_(1),
              sock_(ctx_, ZMQ_PUB)
        {
            sock_.bind(endpoint_);
            spdlog::info("line_zmq_publisher: bound to {}", endpoint_);
        }

        line_zmq_publisher::~line_zmq_publisher()
        {
            sock_.close();
            ctx_.close();
        }

        void line_zmq_publisher::run()
        {
            terminated_ = false;
            auto next_publish = std::chrono::steady_clock::now();

            while (!terminate_requested_)
            {
                auto now = std::chrono::steady_clock::now();
                if (now >= next_publish)
                {
                    try
                    {
                        publish_once();
                    }
                    catch (const std::exception &e)
                    {
                        spdlog::warn("line_zmq_publisher: publish_once threw: {}", e.what());
                    }
                    catch (...)
                    {
                        spdlog::warn("line_zmq_publisher: publish_once threw unknown exception");
                    }
                    next_publish = now + interval_;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            terminated_ = true;
        }

        void line_zmq_publisher::request_terminate()
        {
            terminate_requested_ = true;
        }

        bool line_zmq_publisher::is_terminated() const
        {
            return terminated_;
        }

        // Sample pixels along a 2D line segment in a color image and return avg [R,G,B].
        // Returns false if the image is empty or has no valid pixels.
        static bool avg_line_color(const cv::Mat &img_bgr,
                                   const cv::line_descriptor::KeyLine &kl,
                                   uint8_t &out_r, uint8_t &out_g, uint8_t &out_b)
        {
            if (img_bgr.empty() || img_bgr.channels() < 3)
                return false;

            cv::Point sp(static_cast<int>(kl.startPointX), static_cast<int>(kl.startPointY));
            cv::Point ep(static_cast<int>(kl.endPointX),   static_cast<int>(kl.endPointY));

            cv::LineIterator it(img_bgr, sp, ep, 8);
            if (it.count == 0)
                return false;

            double sum_b = 0, sum_g = 0, sum_r = 0;
            for (int i = 0; i < it.count; ++i, ++it)
            {
                cv::Vec3b px = img_bgr.at<cv::Vec3b>(it.pos());
                sum_b += px[0];
                sum_g += px[1];
                sum_r += px[2];
            }
            out_r = static_cast<uint8_t>(sum_r / it.count);
            out_g = static_cast<uint8_t>(sum_g / it.count);
            out_b = static_cast<uint8_t>(sum_b / it.count);
            return true;
        }

        void line_zmq_publisher::publish_once()
        {
            // ── Line landmarks ────────────────────────────────────────────────
            std::vector<data::Line *> lines;
            map_pub_->get_landmark_lines(lines);

            nlohmann::json lines_arr = nlohmann::json::array();
            for (auto *line : lines)
            {
                if (!line || line->will_be_erased())
                    continue;

                Vec6_t pos = line->get_pos_in_world();

                double sum_r = 0, sum_g = 0, sum_b = 0;
                int color_samples = 0;
                auto observations = line->get_observations();
                for (auto &[keyfrm, keyline_idx] : observations)
                {
                    if (!keyfrm || keyline_idx >= keyfrm->_keylsd.size())
                        continue;
                    cv::Mat img = keyfrm->get_img_rgb();
                    uint8_t r, g, b;
                    if (avg_line_color(img, keyfrm->_keylsd[keyline_idx], r, g, b))
                    {
                        sum_r += r; sum_g += g; sum_b += b;
                        ++color_samples;
                    }
                }

                uint8_t avg_r = 128, avg_g = 128, avg_b = 128;
                if (color_samples > 0)
                {
                    avg_r = static_cast<uint8_t>(sum_r / color_samples);
                    avg_g = static_cast<uint8_t>(sum_g / color_samples);
                    avg_b = static_cast<uint8_t>(sum_b / color_samples);
                }

                lines_arr.push_back({{"id",      line->_id},
                                     {"sp",      {pos[0], pos[1], pos[2]}},
                                     {"ep",      {pos[3], pos[4], pos[5]}},
                                     {"rgb",     {avg_r, avg_g, avg_b}},
                                     {"num_obs", line->num_observations()}});
            }

            // ── Keyframe poses + factor graph edges ───────────────────────────
            std::vector<data::keyframe *> keyfrms;
            map_pub_->get_keyframes(keyfrms);

            nlohmann::json kf_arr   = nlohmann::json::array();
            nlohmann::json edge_arr = nlohmann::json::array();
            nlohmann::json loop_arr = nlohmann::json::array();

            // Track emitted edges to avoid duplicates
            std::set<std::pair<unsigned int, unsigned int>> seen;

            for (auto *kf : keyfrms)
            {
                if (!kf || kf->will_be_erased())
                    continue;

                Mat44_t T_wc = kf->get_cam_pose_inv();
                Vec3_t   pos = T_wc.block<3, 1>(0, 3);
                kf_arr.push_back({{"id",  kf->id_},
                                  {"pos", {pos[0], pos[1], pos[2]}}});

                // Spanning-tree edge (sequential factor graph backbone)
                auto *parent = kf->graph_node_->get_spanning_parent();
                if (parent && !parent->will_be_erased())
                {
                    auto e = std::make_pair(std::min(kf->id_, parent->id_),
                                            std::max(kf->id_, parent->id_));
                    if (seen.insert(e).second)
                        edge_arr.push_back({e.first, e.second});
                }

                // Loop-closure edges (drawn differently in Python)
                for (auto *lkf : kf->graph_node_->get_loop_edges())
                {
                    if (!lkf || lkf->will_be_erased())
                        continue;
                    auto e = std::make_pair(std::min(kf->id_, lkf->id_),
                                            std::max(kf->id_, lkf->id_));
                    if (seen.insert(e).second)
                        loop_arr.push_back({e.first, e.second});
                }
            }

            // ── Current camera position ────────────────────────────────────────
            Mat44_t T_cw = map_pub_->get_current_cam_pose();
            Mat44_t T_wc = T_cw.inverse();
            Vec3_t  cam  = T_wc.block<3, 1>(0, 3);

            nlohmann::json msg = {
                {"lines",     lines_arr},
                {"keyframes", kf_arr},
                {"edges",     edge_arr},
                {"loops",     loop_arr},
                {"cam_pos",   {cam[0], cam[1], cam[2]}}
            };

            auto packed = nlohmann::json::to_msgpack(msg);
            zmq::message_t zmq_msg(packed.data(), packed.size());
            sock_.send(zmq_msg, zmq::send_flags::none);
            spdlog::debug("line_zmq_publisher: published {} lines, {} kfs, {} edges ({} bytes)",
                          lines_arr.size(), kf_arr.size(), edge_arr.size(), packed.size());
        }

    } // namespace publish
} // namespace PLPSLAM
