/* Native bridge test adapter for the actual firmware IMU formatter. */
#include "imu_report.h"
#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif
static IMU_ReportState state;
API void test_init(void) { IMU_ReportInit(&state); }
API unsigned test_raw_size(void) { return sizeof(UART4_JY901_RawData); }
API uint16_t test_format(const UART4_JY901_RawData *raw, uint32_t now,
                         char *buffer, uint16_t capacity)
{
    return IMU_ReportFormat(&state, raw, now, buffer, capacity);
}
API void test_sequence(uint32_t value) { state.sequence = value; }
API void test_yaw_limit(int32_t value) { state.continuous_yaw_cdeg = value; }
