/**
 * This file is part of Structure PLP-SLAM, originally from OpenVSLAM.
 *
 * Copyright 2022 DFKI (German Research Center for Artificial Intelligence)
 * Modified by Fangwen Shu <Fangwen.Shu@dfki.de>
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

#include "kitti_util.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cassert>
#include <stdexcept>

#include <sys/stat.h>

namespace
{
    bool dir_exists(const std::string &path)
    {
        struct stat info;
        return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
    }
}

kitti_sequence::kitti_sequence(const std::string &seq_dir_path)
{
    // load timestamps
    const std::string timestamp_file_path = seq_dir_path + "/times.txt";
    std::ifstream ifs_timestamp;
    ifs_timestamp.open(timestamp_file_path.c_str());
    if (!ifs_timestamp)
    {
        throw std::runtime_error("Could not load a timestamp file from " + timestamp_file_path);
    }

    timestamps_.clear();
    while (!ifs_timestamp.eof())
    {
        std::string s;
        getline(ifs_timestamp, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double timestamp;
            ss >> timestamp;
            timestamps_.push_back(timestamp);
        }
    }

    ifs_timestamp.close();

    // load image file paths
    // KITTI odometry ships two rectified, hardware-synchronised stereo pairs:
    // image_0/image_1 (grayscale) and image_2/image_3 (colour). Prefer the
    // grayscale pair, and fall back to the colour pair for downloads that only
    // carry image_2/image_3. Both pairs share one timestamp per frame index,
    // so pairing by index is the synchronisation.
    std::string left_img_dir_path = seq_dir_path + "/image_0/";
    std::string right_img_dir_path = seq_dir_path + "/image_1/";
    if (!dir_exists(left_img_dir_path))
    {
        left_img_dir_path = seq_dir_path + "/image_2/";
        right_img_dir_path = seq_dir_path + "/image_3/";
        if (!dir_exists(left_img_dir_path))
        {
            throw std::runtime_error("Could not find image_0/ or image_2/ in " + seq_dir_path);
        }
    }

    // KITTI_CAM=3 runs monocular off the right colour camera instead of the
    // left. The two are interchangeable as mono input — same intrinsics, same
    // rectification — so which one gives the better map is an empirical
    // question per sequence, not a fixed choice.
    const char *cam = std::getenv("KITTI_CAM");
    if (cam && std::string(cam) == "3")
    {
        std::swap(left_img_dir_path, right_img_dir_path);
    }

    // KITTI_INTERLEAVE=1 feeds a single monocular stream that alternates
    // between the two colour cameras: even frames from one, odd frames from
    // the other. The camera therefore zig-zags 0.538 m sideways every frame,
    // injecting the lateral parallax that pure forward motion never provides,
    // while the tracker is still never told the baseline — so the map's scale
    // stays as arbitrary as any monocular run.
    const char *inter = std::getenv("KITTI_INTERLEAVE");
    const bool interleave = inter && std::string(inter) == "1";

    left_img_file_paths_.clear();
    right_img_file_paths_.clear();
    for (unsigned int i = 0; i < timestamps_.size(); ++i)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        if (interleave)
        {
            const std::string &dir = (i % 2 == 0) ? left_img_dir_path : right_img_dir_path;
            left_img_file_paths_.push_back(dir + ss.str() + ".png");
            right_img_file_paths_.push_back(right_img_dir_path + ss.str() + ".png");
            continue;
        }
        left_img_file_paths_.push_back(left_img_dir_path + ss.str() + ".png");
        right_img_file_paths_.push_back(right_img_dir_path + ss.str() + ".png");
    }
}

std::vector<kitti_sequence::frame> kitti_sequence::get_frames() const
{
    std::vector<frame> frames;
    assert(timestamps_.size() == left_img_file_paths_.size());
    assert(timestamps_.size() == right_img_file_paths_.size());
    assert(left_img_file_paths_.size() == right_img_file_paths_.size());
    for (unsigned int i = 0; i < timestamps_.size(); ++i)
    {
        frames.emplace_back(frame{left_img_file_paths_.at(i), right_img_file_paths_.at(i), timestamps_.at(i)});
    }
    return frames;
}
