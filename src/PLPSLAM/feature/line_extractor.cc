/**
 * This file is part of Structure PLP-SLAM.
 *
 * Copyright 2022 DFKI (German Research Center for Artificial Intelligence)
 * Developed by Fangwen Shu <Fangwen.Shu@dfki.de>
 *
 * If you use this code, please cite the respective publications as
 * listed on the github repository.
 *
 * Structure PLP-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Structure PLP-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Structure PLP-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include "PLPSLAM/feature/line_extractor.h"
#include <cmath>
#include <numeric>
#include <functional>
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <zmq.h>

// ── ZMQ client for online DeepLSD server ──────────────────────────────────
// Persistent connection: opened once on first use, reused for every frame.
static void   *g_zmq_ctx  = nullptr;
static void   *g_zmq_sock = nullptr;

static bool deeplsd_server_query(const cv::Mat &gray_img, std::string &lines_text)
{
    const char *ep = std::getenv("DEEPLSD_SERVER");
    if (!ep) return false;

    if (!g_zmq_ctx)
    {
        g_zmq_ctx  = zmq_ctx_new();
        g_zmq_sock = zmq_socket(g_zmq_ctx, ZMQ_REQ);
        int timeout_ms = 10000;
        zmq_setsockopt(g_zmq_sock, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
        zmq_connect(g_zmq_sock, ep);
    }

    // Pack: [int32 height][int32 width][H*W uint8 grayscale]
    int32_t h = gray_img.rows, w = gray_img.cols;
    size_t  img_bytes = (size_t)h * w;
    size_t  total     = 8 + img_bytes;
    std::vector<uint8_t> buf(total);
    std::memcpy(buf.data() + 0, &h, 4);
    std::memcpy(buf.data() + 4, &w, 4);
    std::memcpy(buf.data() + 8, gray_img.data, img_bytes);

    zmq_send(g_zmq_sock, buf.data(), total, 0);

    zmq_msg_t reply;
    zmq_msg_init(&reply);
    int rc = zmq_msg_recv(&reply, g_zmq_sock, 0);
    if (rc < 0) { zmq_msg_close(&reply); return false; }

    lines_text.assign(static_cast<const char *>(zmq_msg_data(&reply)),
                      zmq_msg_size(&reply));
    zmq_msg_close(&reply);
    return true;
}

// Merge collinear LSD fragments after LBD has already been computed.
// Modifies segs in-place and selects the corresponding descriptor rows from descr.
// The descriptor of the longest segment in each merge group is kept.
// angle_tol_deg : max orientation difference between two segments to be considered collinear
// dist_tol_px   : max perpendicular distance of one segment's midpoint from the other's line
// gap_tol_px    : max along-axis gap between two segments before they are not merged
static void merge_collinear_lsd(std::vector<cv::line_descriptor::KeyLine> &segs,
                                 cv::Mat &descr,
                                 float angle_tol_deg = 10.0f,
                                 float dist_tol_px   = 4.0f,
                                 float gap_tol_px    = 25.0f)
{
    int n = (int)segs.size();
    if (n < 2) return;

    // Normalize angle to [0, π) so direction is irrelevant
    for (auto &s : segs)
    {
        float a = std::fmod(s.angle, (float)M_PI);
        if (a < 0.0f) a += (float)M_PI;
        s.angle = a;
    }

    float ang_rad = angle_tol_deg * (float)M_PI / 180.0f;

    // Union-Find with path compression
    std::vector<int> par(n);
    std::iota(par.begin(), par.end(), 0);
    std::function<int(int)> find = [&](int x) -> int {
        return par[x] == x ? x : par[x] = find(par[x]);
    };

    for (int i = 0; i < n; i++)
    {
        float dx_i = segs[i].endPointX - segs[i].startPointX;
        float dy_i = segs[i].endPointY - segs[i].startPointY;
        float len_i = std::hypot(dx_i, dy_i);
        if (len_i < 1.0f) continue;
        float ux = dx_i / len_i, uy = dy_i / len_i;
        float nx = -uy, ny = ux;
        float ox = segs[i].startPointX, oy = segs[i].startPointY;

        for (int j = i + 1; j < n; j++)
        {
            // angle check
            float da = std::abs(segs[i].angle - segs[j].angle);
            if (da > (float)M_PI * 0.5f) da = (float)M_PI - da;
            if (da > ang_rad) continue;

            // perpendicular distance of j's midpoint from line i
            float jmx = (segs[j].startPointX + segs[j].endPointX) * 0.5f - ox;
            float jmy = (segs[j].startPointY + segs[j].endPointY) * 0.5f - oy;
            if (std::abs(jmx * nx + jmy * ny) > dist_tol_px) continue;

            // projection gap along i's axis
            float t_js = (segs[j].startPointX - ox) * ux + (segs[j].startPointY - oy) * uy;
            float t_je = (segs[j].endPointX   - ox) * ux + (segs[j].endPointY   - oy) * uy;
            float t_jmin = std::min(t_js, t_je);
            float t_jmax = std::max(t_js, t_je);

            float gap = 0.0f;
            if (t_jmax < 0.0f)       gap = -t_jmax;
            else if (t_jmin > len_i) gap = t_jmin - len_i;
            if (gap > gap_tol_px) continue;

            int ri = find(i), rj = find(j);
            if (ri != rj) par[rj] = ri;
        }
    }

    // Collect groups
    std::unordered_map<int, std::vector<int>> groups;
    for (int i = 0; i < n; i++) groups[find(i)].push_back(i);

    std::vector<cv::line_descriptor::KeyLine> out_segs;
    cv::Mat out_descr;
    out_segs.reserve(groups.size());

    for (auto &[root, members] : groups)
    {
        if ((int)members.size() == 1)
        {
            out_segs.push_back(segs[members[0]]);
            out_descr.push_back(descr.row(members[0]));
            continue;
        }

        // Representative = longest member (its descriptor is kept)
        int rep = members[0];
        for (int m : members)
            if (segs[m].lineLength > segs[rep].lineLength) rep = m;

        float dx = segs[rep].endPointX - segs[rep].startPointX;
        float dy = segs[rep].endPointY - segs[rep].startPointY;
        float len = std::hypot(dx, dy);
        if (len < 1.0f) { out_segs.push_back(segs[rep]); out_descr.push_back(descr.row(rep)); continue; }
        float ux = dx / len, uy = dy / len;
        float ox = segs[rep].startPointX, oy = segs[rep].startPointY;

        // Find extreme projections across all members
        float t_min = 0.0f, t_max = len;
        for (int m : members)
        {
            float ts = (segs[m].startPointX - ox) * ux + (segs[m].startPointY - oy) * uy;
            float te = (segs[m].endPointX   - ox) * ux + (segs[m].endPointY   - oy) * uy;
            t_min = std::min({t_min, ts, te});
            t_max = std::max({t_max, ts, te});
        }

        float x1 = ox + t_min * ux, y1 = oy + t_min * uy;
        float x2 = ox + t_max * ux, y2 = oy + t_max * uy;
        float new_len = t_max - t_min;

        cv::line_descriptor::KeyLine kl = segs[rep];
        kl.startPointX     = x1; kl.startPointY     = y1;
        kl.endPointX       = x2; kl.endPointY       = y2;
        kl.sPointInOctaveX = x1; kl.sPointInOctaveY = y1;
        kl.ePointInOctaveX = x2; kl.ePointInOctaveY = y2;
        kl.pt              = cv::Point2f((x1 + x2) * 0.5f, (y1 + y2) * 0.5f);
        kl.lineLength      = new_len;
        kl.numOfPixels     = (int)std::round(new_len);
        kl.size            = new_len;
        out_segs.push_back(kl);
        out_descr.push_back(descr.row(rep));
    }

    segs  = std::move(out_segs);
    descr = out_descr;
}

namespace PLPSLAM
{
    namespace feature
    {
        LineFeatureTracker::LineFeatureTracker(camera::base *camera)
            : _camera(camera), _frame_cnt(0)
        {
            // this idea from PL-VINS, we only use the line extracted from original image resolution
            _num_levels = 1;
            _scale_factor = 2;

            initialize();
        }

        void LineFeatureTracker::readIntrinsicParameter(const cv::Mat &img)
        {
            auto camera = static_cast<camera::perspective *>(_camera);

            // FW: Get _undist_map1, _undist_map2, _K
            float fx = camera->fx_;
            float fy = camera->fy_;
            float cx = camera->cx_;
            float cy = camera->cy_;
            // cv::Size imageSize = cv::Size(img.cols, img.rows); // (width, height)
            cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F); // rotation matrix, default as Identity matrix

            cv::Mat mapX = cv::Mat::zeros(img.rows, img.cols, CV_32F); // (height, width)
            cv::Mat mapY = cv::Mat::zeros(img.rows, img.cols, CV_32F);

            Eigen::Matrix3d R, R_inv;
            cv::cv2eigen(rmat, R);
            R_inv = R.inverse();

            // assume no skew
            Eigen::Matrix3d K_rect;
            K_rect << fx, 0, cx,
                0, fy, cy,
                0, 0, 1;

            Eigen::Matrix3d K_rect_inv = K_rect.inverse();
            for (int v = 0; v < img.rows; ++v)
            {
                for (int u = 0; u < img.cols; ++u)
                {
                    Eigen::Vector3d xo;
                    xo << u, v, 1;

                    Eigen::Vector3d uo = R_inv * K_rect_inv * xo;
                    Eigen::Vector2d p;
                    const auto z_inv = 1.0 / uo(2);
                    p(0) = fx * uo(0) * z_inv + cx;
                    p(1) = fy * uo(1) * z_inv + cy;

                    mapX.at<float>(v, u) = p(0);
                    mapY.at<float>(v, u) = p(1);
                }
            }

            cv::convertMaps(mapX, mapY, _undist_map1, _undist_map2, CV_32FC1, false);
            cv::eigen2cv(K_rect, _K);
        }

        void LineFeatureTracker::extract_LSD_LBD(const cv::Mat &img,
                                                 std::vector<cv::line_descriptor::KeyLine> &frame_keylsd,
                                                 cv::Mat &frame_lbd_descr,
                                                 std::vector<Vec3_t> &keyline_functions)
        {
            readIntrinsicParameter(img);
            cv::Mat img_temp;
            _frame_cnt++;

            cv::remap(img, img_temp, _undist_map1, _undist_map2, cv::INTER_LINEAR);

            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clache = cv::createCLAHE(3.0, cv::Size(8, 8));
                clache->apply(img_temp, img_temp);
            }

            std::vector<cv::line_descriptor::KeyLine> lsd, keylsd;
            cv::Mat lbd_descr, keylbd_descr;
            cv::Ptr<cv::line_descriptor::BinaryDescriptor> binary_descriptor =
                cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();

            // [1] DeepLSD — online server (DEEPLSD_SERVER) or file cache (DEEPLSD_DIR)
            bool used_deeplsd = false;

            auto parse_deeplsd_text = [&](const std::string &text) {
                std::istringstream ss(text);
                float x1, y1, x2, y2;
                while (ss >> x1 >> y1 >> x2 >> y2)
                {
                    float dx = x2 - x1, dy = y2 - y1;
                    float len = std::hypot(dx, dy);
                    if (len < 15.0f) continue;
                    cv::line_descriptor::KeyLine kl;
                    kl.startPointX     = x1; kl.startPointY     = y1;
                    kl.endPointX       = x2; kl.endPointY       = y2;
                    kl.sPointInOctaveX = x1; kl.sPointInOctaveY = y1;
                    kl.ePointInOctaveX = x2; kl.ePointInOctaveY = y2;
                    kl.pt              = cv::Point2f((x1 + x2) * 0.5f, (y1 + y2) * 0.5f);
                    kl.lineLength      = len;
                    kl.numOfPixels     = (int)std::round(len);
                    kl.angle           = std::atan2(dy, dx);
                    kl.octave          = 0;
                    kl.size            = len;
                    kl.response        = 1.0f;
                    kl.class_id        = (int)lsd.size();
                    lsd.push_back(kl);
                }
            };

            // 1a. Online ZMQ server
            if (std::getenv("DEEPLSD_SERVER"))
            {
                std::string text;
                if (deeplsd_server_query(img_temp, text))
                {
                    parse_deeplsd_text(text);
                    used_deeplsd = true;
                }
            }

            // 1b. Pre-computed file cache (DEEPLSD_DIR=path/to/cache)
            if (!used_deeplsd)
            {
                const char *deeplsd_dir_env = std::getenv("DEEPLSD_DIR");
                if (deeplsd_dir_env)
                {
                    char filepath[4096];
                    std::snprintf(filepath, sizeof(filepath), "%s/%06d.txt",
                                  deeplsd_dir_env, _frame_cnt - 1);
                    std::ifstream f(filepath);
                    if (f.is_open())
                    {
                        std::string text((std::istreambuf_iterator<char>(f)),
                                          std::istreambuf_iterator<char>());
                        parse_deeplsd_text(text);
                        used_deeplsd = true;
                    }
                }
            }

            if (!used_deeplsd)
            {
                // [1c] LSD fallback with tuned params
                cv::Ptr<cv::line_descriptor::LSDDetectorC> lsd_detector =
                    cv::line_descriptor::LSDDetectorC::createLSDDetectorC();
                cv::line_descriptor::LSDDetectorC::LSDOptions opts;
                opts.refine      = 1;
                opts.scale       = 0.8;
                opts.sigma_scale = 0.8;
                opts.quant       = 2.0;
                opts.ang_th      = 45.0;
                opts.log_eps     = 0.0;
                opts.density_th  = 0.3;
                opts.n_bins      = 1024;
                opts.min_length  = _min_line_length * (std::min(img_temp.cols, img_temp.rows));
                lsd_detector->detect(img_temp, lsd, _scale_factor, _num_levels, opts);
            }

            // [2] LBD descriptors
            binary_descriptor->compute(img_temp, lsd, lbd_descr);

            // [3] keep octave-0 lines above length threshold
            // DeepLSD: 15px (NFA already filters noise); LSD fallback: 20px
            const float min_len = used_deeplsd ? 15.0f : 15.0f;
            for (unsigned int i = 0; i < lsd.size(); i++)
            {
                if (lsd[i].octave == 0 && lsd[i].lineLength >= min_len)
                {
                    keylsd.push_back(lsd[i]);
                    keylbd_descr.push_back(lbd_descr.row(i));
                }
            }

            // [4] merge collinear fragments (LSD only; DeepLSD already merges internally)
            if (!used_deeplsd)
            {
                merge_collinear_lsd(keylsd, keylbd_descr,
                                    10.0f, 3.0f, 25.0f);
            }

            frame_keylsd = keylsd;
            frame_lbd_descr = keylbd_descr;

            // [3] calculate the parameters (functions) of the 2D line segments extracted in the image, later on used for triangulation
            for (std::vector<cv::line_descriptor::KeyLine>::iterator it = frame_keylsd.begin(); it != frame_keylsd.end(); ++it)
            {
                Vec3_t sp_l;
                sp_l << it->startPointX, it->startPointY, 1.0; // (x, y, 1) starting point

                Vec3_t ep_l;
                ep_l << it->endPointX, it->endPointY, 1.0; // (x, y, 1) ending point

                Vec3_t lineFunction;              // line function in 2D image, e.g. ax + by + c = 0
                lineFunction << sp_l.cross(ep_l); // cross product gives the normal vector
                lineFunction = lineFunction / sqrt(lineFunction(0) * lineFunction(0) + lineFunction(1) * lineFunction(1));
                keyline_functions.push_back(lineFunction);
            }
        }

        void LineFeatureTracker::initialize()
        {
            // calculate scale_factors at each level of image pyramid
            std::vector<float> scale_factors(_num_levels, 1.0);
            if (_num_levels > 1)
            {
                for (unsigned int level = 1; level < _num_levels; ++level)
                {
                    scale_factors.at(level) = _scale_factor * scale_factors.at(level - 1);
                }
                _scale_factors = scale_factors;
            }
            else
            {
                _scale_factors = scale_factors;
            }

            // calculate inv_scale_factors
            std::vector<float> inv_scale_factors(_num_levels, 1.0);
            if (_num_levels > 1)
            {
                for (unsigned int level = 1; level < _num_levels; ++level)
                {
                    inv_scale_factors.at(level) = (1.0f / _scale_factor) * inv_scale_factors.at(level - 1);
                }
                _inv_scale_factors = inv_scale_factors;
            }
            else
            {
                _inv_scale_factors = inv_scale_factors;
            }

            // calculate level_sigma_sq
            std::vector<float> level_sigma_sq(_num_levels, 1.0);
            float scale_factor_at_level = 1.0;
            if (_num_levels > 1)
            {
                for (unsigned int level = 1; level < _num_levels; ++level)
                {
                    scale_factor_at_level = _scale_factor * scale_factor_at_level;
                    level_sigma_sq.at(level) = scale_factor_at_level * scale_factor_at_level;
                }
                _level_sigma_sq = level_sigma_sq;
            }
            else
            {
                _level_sigma_sq = level_sigma_sq;
            }

            // calculate inv_level_sigma_sq
            std::vector<float> inv_level_sigma_sq(_num_levels, 1.0);
            scale_factor_at_level = 1.0;
            if (_num_levels > 1)
            {
                for (unsigned int level = 1; level < _num_levels; ++level)
                {
                    scale_factor_at_level = _scale_factor * scale_factor_at_level;
                    inv_level_sigma_sq.at(level) = 1.0f / (scale_factor_at_level * scale_factor_at_level);
                }
                _inv_level_sigma_sq = inv_level_sigma_sq;
            }
            else
            {
                _inv_level_sigma_sq = inv_level_sigma_sq;
            }
        }

        unsigned int LineFeatureTracker::get_num_scale_levels() const
        {
            return _num_levels;
        }

        float LineFeatureTracker::get_scale_factor() const
        {
            return _scale_factor;
        }

        std::vector<float> LineFeatureTracker::get_scale_factors() const
        {
            return _scale_factors;
        }

        std::vector<float> LineFeatureTracker::get_inv_scale_factors() const
        {
            return _inv_scale_factors;
        }

        std::vector<float> LineFeatureTracker::get_level_sigma_sq() const
        {
            return _level_sigma_sq;
        }

        std::vector<float> LineFeatureTracker::get_inv_level_sigma_sq() const
        {
            return _inv_level_sigma_sq;
        }
    }
}