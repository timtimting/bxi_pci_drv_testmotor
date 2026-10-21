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
static unsigned int master_query_calls;
static unsigned int activation_order;
int ecrt_master_application_time(ec_master_t *m, uint64_t t) { (void)m; (void)t; assert(activation_order==0); activation_order=1; return 0; }
int ecrt_master_activate(ec_master_t *m) { (void)m; assert(activation_order==1); activation_order=2; return 0; }
int ecrt_master(ec_master_t *m, ec_master_info_t *i) { (void)m; memset(i,0,sizeof(*i)); i->slave_count=2; master_query_calls++; return 0; }
int ecrt_master_get_slave(ec_master_t *m, uint16_t p, ec_slave_info_t *i) {
 (void)m; memset(i,0,sizeof(*i)); i->alias=42;
 if (master_query_calls > 1 || p == 0) { i->vendor_id=0x00010203; i->product_code=0x00000402; }
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
 char configured_devices[] = "48:21:0b:72:f7:70,6c:1f:f7:c5:03:f3\n";
 char second_config[] = "48:21:0B:72:F7:70,6C:1F:F7:C5:03:F3\n";
 assert(etherlab_master_index_for_mac(configured_devices, "6c:1f:f7:c5:03:f3") == 1);
 assert(etherlab_master_index_for_mac(second_config, "48:21:0b:72:f7:70") == 0);
 assert(etherlab_master_index_for_mac(second_config, "6c:1f:f7:c5:03:f4") == -1);
 etherlab_master = (ec_master_t *)1;
 ec_slavecount=1; assert(etherlab_safe_op_timeout_us()==7000000);
 ec_slavecount=2; assert(etherlab_safe_op_timeout_us()==22000000);
 ec_slavecount=3; assert(etherlab_safe_op_timeout_us()==37000000);
 ec_slavecount=4; assert(etherlab_safe_op_timeout_us()==52000000);
 ec_slavecount=255; assert(etherlab_safe_op_timeout_us()==120000000);
 assert(ec_config_init(FALSE)==2); assert(config_calls==2); assert(master_query_calls==2);
 assert(etherlab_slaves[1].config != etherlab_slaves[2].config);
 assert(etherlab_activate()==0); assert(activation_order==2);
 uint8_t outputs[2][23], inputs[2][16] = {{0}};
 uint8_t selected[EC_MAXSLAVE] = {0};
 int32_t ignored_targets[EC_MAXSLAVE] = {0};
 const float positions[2] = {1.25f, -0.75f};
 ethercat_mit_mapping = true;
 memset(outputs, 0xa5, sizeof(outputs));
 for (int slave = 1; slave <= 2; ++slave) {
  selected[slave] = 1;
  ec_slave[slave].outputs = outputs[slave - 1];
  ec_slave[slave].inputs = inputs[slave - 1];
  ec_slave[slave].Obits = 184;
  ec_slave[slave].Ibits = 128;
  console_ethercat_write_float(inputs[slave - 1] + 2, positions[slave - 1]);
 }
 assert(console_ethercat_selected_ready(selected, ignored_targets, ETHERCAT_CIA402_MODE_MIT) == 0);
 for (int slave = 1; slave <= 2; ++slave) {
  uint8_t *out = outputs[slave - 1];
  assert(console_ethercat_read_u16(out) == 0); /* No servo enable during warm-up. */
  assert(out[22] == ETHERCAT_CIA402_MODE_MIT);
  assert(console_ethercat_read_float(out + 2) == positions[slave - 1]);
  for (int offset = 6; offset <= 18; offset += 4)
   assert(console_ethercat_read_float(out + offset) == 0.0f);
 }
 puts("PASS: two-slave MIT warm-up preserves feedback positions, zeroes gains/feedforward, stays disabled");
 puts("PASS: waits for nonzero slave identities before configuring absolute positions 0/1");
 puts("PASS: initializes EtherLab application time before master activation");
 return 0;
}
