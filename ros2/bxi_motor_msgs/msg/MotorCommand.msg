uint8 ENABLE=1
uint8 DISABLE=2
uint8 MIT=3
uint8 PN082=4
uint8 SAVE=5
uint8 QUALITY=6
uint8 POWER_ON=7
uint8 POWER_OFF=8
uint8 action
# EtherCAT console slave ID: 1..255; 0 selects all slaves.
uint16 slave_id
float32 position
float32 velocity
float32 torque
float32 kp
float32 kd
# MIT/ENABLE: 0 selects node default_hold_ms. QUALITY: test duration.
uint32 duration_ms
# PN082 raw integer encoding, not the catalogue Nm/Arms coefficient.
uint16 pn082
