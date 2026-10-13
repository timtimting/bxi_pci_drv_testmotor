#define main runtime_embedded_main
#include "../src/lib/runtime.c"
#undef main
#include "../src/lib/ethercat_output.c"
#undef printf
#define printf(...) console_ethercat_printf(__VA_ARGS__)
#include "../src/lib/ethercat_scan.c"
#include "../src/lib/ethercat_background.c"
#undef printf
#include <assert.h>

static unsigned int config_calls;
int ecrt_master(ec_master_t *m, ec_master_info_t *i) { (void)m; memset(i,0,sizeof(*i)); i->slave_count=2; return 0; }
int ecrt_master_get_slave(ec_master_t *m, uint16_t p, ec_slave_info_t *i) {
 (void)m; memset(i,0,sizeof(*i)); i->alias=42; i->vendor_id=0x00010203; i->product_code=0x00000402;
 i->al_state=EC_STATE_PRE_OP; snprintf(i->name,sizeof(i->name),"mock%u",p); return 0;
}
ec_slave_config_t *ecrt_master_slave_config(ec_master_t *m, uint16_t a, uint16_t p, uint32_t v, uint32_t product) {
 (void)m; assert(a==0); assert(p==config_calls++); assert(v==0x00010203 && product==0x00000402);
 return (ec_slave_config_t *)(uintptr_t)(p+1);
}
ec_reg_request_t *ecrt_slave_config_create_reg_request(ec_slave_config_t *c,size_t n) { (void)c; (void)n; return (ec_reg_request_t *)1; }
ec_sdo_request_t *ecrt_slave_config_create_sdo_request(ec_slave_config_t *c,uint16_t i,uint8_t sub,size_t n) { (void)c; (void)i; (void)sub; (void)n; return (ec_sdo_request_t *)1; }
int ecrt_sdo_request_timeout(ec_sdo_request_t *r,uint32_t t) { (void)r; assert(t==700); return 0; }
int main(void) {
 etherlab_master = (ec_master_t *)1;
 ec_slavecount=1; assert(etherlab_safe_op_timeout_us()==7000000);
 ec_slavecount=2; assert(etherlab_safe_op_timeout_us()==14000000);
 ec_slavecount=3; assert(etherlab_safe_op_timeout_us()==21000000);
 ec_slavecount=255; assert(etherlab_safe_op_timeout_us()==60000000);
 assert(ec_config_init(FALSE)==2); assert(config_calls==2);
 assert(etherlab_slaves[1].config != etherlab_slaves[2].config);
 puts("PASS: duplicate nonzero aliases still map to absolute positions 0/1; per-slave timeout and cap");
 return 0;
}
