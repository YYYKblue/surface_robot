/* Host test adapter: compile the actual H7 nano_comm.c and motor.c unchanged.
 * Only UART, motor pins, time, and the independent RC owner's input are mocked. */
#include <stdint.h>
#include <string.h>
#include "nano_comm.h"
#include "nano_port.h"
#include "remote_control.h"
#include "motor.h"
#include "motor_hw.h"

static uint32_t now, last_rx;
static uint8_t rx[1024];
static unsigned head, tail;
static char tx[1024];
static unsigned tx_size;
RemoteState g_remote;
int NanoPort_Init(void) { return 1; }
int NanoPort_ReadByte(uint8_t *byte, uint32_t *at) {
    if (head == tail) return 0;
    *byte = rx[tail++]; *at = last_rx; return 1;
}
int NanoPort_TakeFault(void) { return 0; }
int NanoPort_Send(const char *data, uint16_t size) {
    if (size > sizeof(tx) - tx_size) return 0;
    memcpy(tx + tx_size, data, size); tx_size += size; return 1;
}
int NanoPort_TxReady(void) { return 1; }
uint32_t NanoPort_NowMs(void) { return now; }
uint32_t NanoPort_RxQuietMs(void) { return now - last_rx; }
int MotorHw_Init(void) { return 1; }
void MotorHw_PrepareSafePins(void) {}
void MotorHw_SetBldc(MotorId id, uint8_t fr, uint8_t duty) {(void)id;(void)fr;(void)duty;}
void MotorHw_SetTopPulse(uint16_t pulse) {(void)pulse;}
void MotorHw_EmergencyStop(void) {}
uint32_t MotorHw_NowMs(void) { return now; }
const char *Remote_Result(int prefix) {(void)prefix; return "RC_OK";}
void test_init(void) { Motor_Init(); NanoComm_Init(); }
void test_tick(uint32_t at) { now=at; NanoComm_Process(); Motor_Process(); }
void test_feed(const uint8_t *data, unsigned size, uint32_t at) {
    now=last_rx=at; head=size; tail=0; memcpy(rx,data,size); NanoComm_Process();
}
unsigned test_read(char *out) { unsigned n=tx_size; memcpy(out,tx,n); tx_size=0; return n; }
unsigned test_output(unsigned id) {
    MotorState state; Motor_GetState((MotorId)id,&state); return state.output_percent;
}
unsigned test_fr(unsigned id) {
    MotorState state; Motor_GetState((MotorId)id,&state); return state.fr_high;
}
unsigned test_top(void) {
    MotorState state; Motor_GetState(MOTOR_TOP,&state); return state.top_pulse_us;
}
void test_remote(unsigned owner) {
    NanoComm_SetRemoteOwner((uint8_t)owner);
    g_remote.owner=(uint8_t)owner;g_remote.armed=1;g_remote.direction=3;g_remote.speed=40;
    if (owner) Motor_SetMotion(MOTOR_DIR_LEFT,40);
}
void test_reboot_session(uint32_t at) { now=at; NanoComm_Init(); }
