#ifndef _SPARSE_STEREO_PARAMS_
#define _SPARSE_STEREO_PARAMS_

#include <iostream>
#include <memory>

namespace pips_sparse_stereo{
struct matching_params {

public:
    double maxDisparity, maxDistance, range;
    int epiRange, k, detector_method, descriptor_method, orb_num;
    bool knn_enabled, repeated_remove, useBFMatcher;
};

typedef std::shared_ptr<matching_params> matching_params_ptr;
}

#endif //_SPARSE_STEREO_PARAMS_
