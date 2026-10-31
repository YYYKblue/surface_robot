#ifndef __LDRALGORITHM_H__
#define __LDRALGORITHM_H__

#include <vector>
#include <algorithm>
#include <cmath>
#include "pavo2_types.h"

namespace  pavo2 {

const int INTENSITY_THRESHOLD_TABLE_LENGTH = 24;
const int INTENSITY_MAX = 2000;
const int FILTER_WINDOW_SIZE = 5;

class LdrAlgorithm
{
public:
    LdrAlgorithm();
    void InitTables();

    void FarHighRelectivityPreProcess(pavo_response_scan_t* points, int size);
    void FarHighRelectivityFilter(pavo_response_scan_t* points, int size);

    static float AngleCos(float A1, float B1, float A2, float B2);

private:
    std::vector<float> m_intensity_threshold_table;

    std::vector<double> m_cos_lookup_table;
    std::vector<double> m_sin_lookup_table;
};

} //namespace  pavo2

#endif // __LDRALGORITHM_H__
