#ifndef GCC13_FIX_H
#define GCC13_FIX_H

#include <memory>
#include <iomanip>

// Fix for 'number_t' errors
using number_t = double;

// Fix for OpenCV 4 constants
#include <opencv2/imgproc.hpp>
#ifndef CV_INTER_LINEAR
#define CV_INTER_LINEAR cv::INTER_LINEAR
#endif

// Fix for modern g2o missing make_unique
namespace g2o {
    template<typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}

#endif