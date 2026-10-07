#ifndef ETHERLAB_COMPAT_H
#define ETHERLAB_COMPAT_H

#include <ecrt.h>
#include <stdbool.h>
#include <stdint.h>

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int boolean;

#define TRUE 1
#define FALSE 0
#define EC_MAXSLAVE 256
#define EC_TIMEOUTRET 2000
#define EC_TIMEOUTRXM 700000
#define EC_TIMEOUTSTATE 100000
#define EC_STATE_INIT 0x01
#define EC_STATE_PRE_OP 0x02
#define EC_STATE_SAFE_OP 0x04
#define EC_STATE_OPERATIONAL 0x08
#define EC_STATE_BOOT 0x03

typedef struct {
    char name[EC_MAX_STRING_LENGTH];
    uint16 configadr;
    uint32_t eep_man;
    uint32_t eep_id;
    uint32_t eep_rev;
    uint8 state;
    uint16 ALstatuscode;
    uint16 Obytes;
    uint16 Ibytes;
    uint16 Obits;
    uint16 Ibits;
    uint8 hasdc;
    uint8 DCactive;
    uint32_t DCcycle;
    int32_t DCshift;
    void *outputs;
    void *inputs;
} ec_compat_slave;

typedef struct {
    int outputsWKC;
    int inputsWKC;
} ec_compat_group;

extern ec_compat_slave ec_slave[EC_MAXSLAVE];
extern ec_compat_group ec_group[1];
extern int ec_slavecount;
extern int64_t ec_DCtime;

int ec_init(char *interface);
int ec_config_init(boolean use_table);
int ec_config_map(void *process_image);
int ec_configdc(void);
int ec_dcsync0(uint16 slave, boolean activate, uint32_t cycle_ns, int32_t shift_ns);
int ec_send_processdata(void);
int ec_receive_processdata(int timeout_us);
int ec_statecheck(uint16 slave, uint16 requested_state, int timeout_us);
int ec_readstate(void);
int ec_writestate(uint16 slave);
int ec_close(void);
int ec_FPRD(uint16 configadr, uint16 reg, uint16 length, void *data, int timeout_us);
int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean complete_access,
               int *size, void *data, int timeout_us);
int ec_SDOwrite(uint16 slave, uint16 index, uint8 subindex, boolean complete_access,
                int size, const void *data, int timeout_us);

#endif
