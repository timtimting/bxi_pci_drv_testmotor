/* Hardware-free transport tests. Link with --wrap=canfd_send_packet. */
#define MOTOR_CONSOLE_TEST
#include "../src/main.c"
#include <assert.h>

static flash_state fixture;
static unsigned int sends[3];
static int missing=-1;
static bool wrong_bus;

int __wrap_canfd_send_packet(canfd_packet *packet, unsigned int count)
{
    assert(count==1 && packet->frame.len==8);
    int index=packet->bus==0 ? 0 : packet->frame.can_id==1 ? 1 : 2;
    ++sends[index];
    if (index==missing) return 0;
    rx_can_frame frame={0};
    frame.bus=wrong_bus ? 7 : packet->bus;
    frame.can_id=0x10|packet->frame.can_id;
    frame.len=8; frame.time_us=time_us();
    frame.data[0]=packet->frame.can_id;
    /* Give each member a distinct decoded position. */
    frame.data[1]=0x80+index; frame.data[3]=frame.data[6]=0x80;
    frame_ring_push(&fixture.frames,&frame);
    return 0;
}

int main(void)
{
    fixture.config.entry_count=3;
    fixture.config_loaded=true;
    for (size_t i=0;i<3;++i) {
        fixture.config.entries[i].bus=i ? 1 : 0;
        fixture.config.entries[i].id=i==2 ? 2 : 1;
    }
    fixture.config.mit_limits=bxi_motor_default_limits;
    fixture.quiet_tx=true; fixture.debug_use_canfd=true;
    frame_ring_init(&fixture.frames);
    zero_group_motor motors[32]; size_t count;
    assert(!zero_group_select(&fixture,"0:1,1:1,1:2",motors,&count) && count==3);
    assert(zero_group_select(&fixture,"0:1,0:1",motors,&count));
    assert(zero_group_select(&fixture,"0:1,",motors,&count));
    assert(zero_group_select(&fixture,"4:9",motors,&count));
    assert(!zero_group_select(&fixture,"0:1,1:1,1:2",motors,&count));
    char automatic[512];
    for (size_t i=0;i<3;++i) {
        strcpy(fixture.config.entries[i].type,"85");
        fixture.motors[i].online=true;
        update_motor_detected_type_from_line(&fixture.motors[i],"HW    : Bxi_motor_85");
    }
    assert(!zero_group_discover(&fixture,automatic,sizeof(automatic)));
    assert(!strcmp(automatic,"0:1,1:1,1:2"));
    fixture.motors[1].online=false;
    assert(!zero_group_discover(&fixture,automatic,sizeof(automatic)));
    assert(!strcmp(automatic,"0:1,1:2"));
    fixture.motors[1].online=true;
    update_motor_detected_type_from_line(&fixture.motors[1],"PRO   : motor_70");
    assert(zero_group_discover(&fixture,automatic,sizeof(automatic)));
    fixture.motors[1].detected_type_valid=false;
    assert(zero_group_discover(&fixture,automatic,sizeof(automatic)));
    update_motor_detected_type_from_line(&fixture.motors[1],"PRO   : motor_85");
    float positions[3]={.1,.1,.1};
    assert(!zero_group_exchange(&fixture,motors,count,positions,NULL,50,5));
    assert(motors[0].reply.position<motors[1].reply.position && motors[1].reply.position<motors[2].reply.position);
    memset(sends,0,sizeof(sends)); missing=1;
    assert(zero_group_exchange(&fixture,motors,count,positions,NULL,50,5));
    /* A missing motor cannot starve communication to either healthy member. */
    assert(sends[0]>=5 && sends[0]==sends[1] && sends[1]==sends[2]);
    missing=-1; wrong_bus=true;
    assert(zero_group_exchange(&fixture,motors,count,positions,NULL,50,5));
    zero_cycle_config config; assert(!zero_cycle_load("config/zero_cycle_test.yaml",&config));
    double scale=2*M_PI/config.ratio;
    motors[0].before=scale; motors[0].reply.position=7*scale;
    bool frozen=false;
    assert(!zero_group_compare(&motors[0],&config,&frozen) && frozen);
    /* Identical wrong readings on a second reboot still fail against the
     * trusted original baseline, and never clear the movement latch. */
    assert(!zero_group_compare(&motors[0],&config,&frozen));
    assert(fabs(motors[0].before-scale)<1e-6 && motors[0].failed==2);
    motors[0].reply.position=scale;
    assert(zero_group_compare(&motors[0],&config,&frozen) && frozen);
    puts("PASS: group selection, bus/id routing, cyclic keepalive, timeout, frozen baseline and latched motion stop");
    return 0;
}
