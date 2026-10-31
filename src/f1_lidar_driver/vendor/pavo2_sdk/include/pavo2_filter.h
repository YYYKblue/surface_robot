#ifndef  __PAVO2_FILTER_H__
#define  __PAVO2_FILTER_H__

#include <cstdint>
#include <vector>
#include "pavo2_types.h"

namespace pavo2
{

class MADataFilter
{
public:
    MADataFilter();
    pavo_response_scan_t* Filter(pavo_response_scan_t* data_buffer, int count);
    void EnableTailFilter(int method);
    void EnableFrontFilter(int level);
private:  
    pavo_response_scan_t*  HandlTialFilter(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithRatio(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithDistance_1(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithDistance_2(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithDistanceForContinuity(pavo_response_scan_t* data_buffer, int count);

    pavo_response_scan_t*  HandlTialFilterWithAngle(pavo_response_scan_t* data_buffer, int count);

    void HandlFrontFilter(pavo_response_scan_t* data_buffer, int count);

    uint16_t SearchSkipPoint(pavo_response_scan_t* data_buffer, int idx, int capacity);

    uint16_t SearchContinuityPoint(pavo_response_scan_t* data_buffer, int idx, int capacity);

    uint16_t HandlePoint(pavo_response_scan_t* data_buffer, int start, int end, int capacity);

    void SetWindow(int size);

    void SetDistance(double size);

private:
    uint16_t m_window_size;
    double   m_distance_diff;
    std::vector<int> m_window_vector;
    int m_tailfilter_method;
    int m_tailfilter_method_angle;
    int front_filter_level;
};

} //namespace pavo2

#endif // ! __PAVO2_FILTER_H__
