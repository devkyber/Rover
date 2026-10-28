#include "control.h"
#include <limits.h>
#include <string.h>

static bool fresh(const scout_control_t *c,uint64_t now)
{ return c->linked && now>=c->heartbeat_us && now-c->heartbeat_us<500000; }
static int64_t delta(int32_t value,int32_t origin)
{
    uint32_t d=(uint32_t)value-(uint32_t)origin;
    return d<=INT32_MAX?(int64_t)d:(int64_t)d-INT64_C(4294967296);
}
static bool motor_active(const scout_control_t *c)
{ return c->state==CT_LOADING || c->state==CT_UNLOADING; }
static bool powered(const scout_control_t *c)
{ return motor_active(c) || c->last_servo_us!=0; }
static bool command_ready(const scout_control_t *c,uint64_t now,bool healthy)
{ return c->configured && fresh(c,now) && healthy; }
static void clear_stop_for_action(scout_control_t *c)
{
    if (c->state==CT_FAULT) c->state=CT_DISARMED;
    c->fault="none"; c->needs_reset=false; c->boot_confirmation_required=false;
}
void scout_control_init(scout_control_t *c)
{
    memset(c,0,sizeof(*c)); c->state=CT_DISARMED; c->fault="none";
    c->mechanism=(scout_mechanism_t){.motor_direction=1,.encoder_sign=1,.motor_percent=30,
        .servo_mode=1,.servo_prepare_us=SCOUT_SERVO_PULL_US,
        .servo_latch_us=SCOUT_SERVO_LOCK_US};
    c->configured=true; /* Valid manual defaults; every output still starts off. */
}
bool scout_control_busy(const scout_control_t *c)
{ return powered(c); }
bool scout_mechanism_valid(const scout_mechanism_t *m)
{
    return m && (m->motor_direction==1 || m->motor_direction==-1) &&
        (m->encoder_sign==1 || m->encoder_sign==-1) &&
        m->motor_percent>=1 && m->motor_percent<=100 &&
        m->load_timeout_ms==0 && m->stall_ms==0 && m->hold_tolerance_counts==0 &&
        m->servo_mode==1 && m->servo_neutral_us==0 &&
        m->servo_prepare_us==SCOUT_SERVO_PULL_US && m->servo_latch_us==SCOUT_SERVO_LOCK_US &&
        m->prepare_ms==0 && m->latch_ms==0 && m->release_ms==0;
}
bool scout_control_configure(scout_control_t *c,const scout_mechanism_t *m)
{
    if (motor_active(c) || !scout_mechanism_valid(m)) return false;
    c->mechanism=*m; c->configured=true;
    return true; /* Configuration does not issue or cancel a servo position. */
}
void scout_control_stop(scout_control_t *c,const char *reason,uint64_t now)
{
    (void)now;
    if (c->state==CT_FAULT) return; /* Keep the first cause until a successful new button. */
    if (c->state==CT_LOADING) {
        c->compression_result_counts=c->progress_counts; c->compression_result_valid=true;
    }
    c->state=CT_FAULT; c->fault=reason?reason:"operator_stop";
    c->last_servo_us=0; c->servo_role=SERVO_ROLE_OFF;
    c->needs_reset=false; c->boot_confirmation_required=false; c->neutral_until_us=0;
}
void scout_control_heartbeat(scout_control_t *c,uint64_t now)
{
    if (c->linked && !fresh(c,now) && powered(c))
        scout_control_stop(c,"link_timeout",now);
    c->linked=true; c->heartbeat_us=now;
}
bool scout_control_clear_fault(scout_control_t *c,bool healthy,bool mechanism_reset)
{
    (void)mechanism_reset;
    if (!healthy || c->state!=CT_FAULT) return false;
    clear_stop_for_action(c);
    return true; /* Compatibility only; ordinary buttons can recover directly. */
}
static bool servo_position(scout_control_t *c,scout_servo_role_t role,unsigned pulse,uint64_t now,bool healthy)
{
    if (!command_ready(c,now,healthy)) return false;
    clear_stop_for_action(c);
    c->servo_role=role; c->last_servo_us=pulse;
    return true; /* Preserve either DC direction and the forward encoder measurement. */
}
bool scout_control_prepare(scout_control_t *c,uint64_t now,bool healthy)
{ return servo_position(c,SERVO_ROLE_PULL,c->mechanism.servo_prepare_us,now,healthy); }
bool scout_control_latch(scout_control_t *c,uint32_t cycle_id,uint64_t now,bool healthy)
{
    (void)cycle_id;
    return servo_position(c,SERVO_ROLE_LOCK,c->mechanism.servo_latch_us,now,healthy);
}
bool scout_control_launch(scout_control_t *c,uint64_t now,bool healthy)
{ return servo_position(c,SERVO_ROLE_LAUNCH,SCOUT_SERVO_LAUNCH_US,now,healthy); }
bool scout_control_load(scout_control_t *c,uint64_t now,int32_t encoder,bool healthy)
{
    if (!command_ready(c,now,healthy)) return false;
    if (c->state==CT_LOADING) return true; /* Repeat start never resets a running measurement. */
    clear_stop_for_action(c);
    c->cycle_id=c->cycle_id==UINT32_MAX?1:c->cycle_id+1;
    c->origin_encoder=encoder; c->progress_counts=c->best_progress=0;
    c->compression_result_valid=false; c->compression_result_counts=0;
    c->state=CT_LOADING; c->state_us=c->progress_us=now;
    return true;
}
bool scout_control_unload(scout_control_t *c,uint64_t now,int32_t encoder,bool healthy)
{
    if (!command_ready(c,now,healthy)) return false;
    if (c->state==CT_UNLOADING) return true; /* Repeated reverse requests change nothing. */
    /* A direction change ends the forward measurement before reverse travel begins. */
    if (c->state==CT_LOADING) scout_control_compression_stop(c,now,encoder);
    clear_stop_for_action(c);
    c->state=CT_UNLOADING; c->state_us=now;
    return true;
}
bool scout_control_compression_stop(scout_control_t *c,uint64_t now,int32_t encoder)
{
    if (c->state==CT_UNLOADING) {
        c->state=CT_DISARMED; c->state_us=now;
        return true; /* Reverse travel must not replace the last forward measurement. */
    }
    if (c->state!=CT_LOADING) return true; /* Always accepted; preserve earlier measurements. */
    c->progress_counts=delta(encoder,c->origin_encoder)*c->mechanism.encoder_sign;
    c->compression_result_counts=c->progress_counts; c->compression_result_valid=true;
    c->loaded_encoder=encoder; c->state=CT_LOADED; c->state_us=now;
    return true; /* Only DC stops. Servo role and held position remain exactly as requested. */
}
scout_outputs_t scout_control_tick(scout_control_t *c,uint64_t now,int32_t encoder,bool healthy)
{
    if (c->state==CT_LOADING)
        c->progress_counts=delta(encoder,c->origin_encoder)*c->mechanism.encoder_sign;
    if (powered(c)) {
        if (!healthy) scout_control_stop(c,"hardware_fault",now);
        else if (!fresh(c,now)) scout_control_stop(c,"link_timeout",now);
    }
    if (c->state==CT_FAULT) return (scout_outputs_t){0};
    return (scout_outputs_t){
        .motor=c->state==CT_LOADING?c->mechanism.motor_direction:
            c->state==CT_UNLOADING?-c->mechanism.motor_direction:0,
        .motor_percent=motor_active(c)?c->mechanism.motor_percent:0,
        .loading_enabled=c->last_servo_us!=0,.loading_us=c->last_servo_us};
}
const char *scout_control_name(scout_control_state_t s)
{
    static const char *names[]={"idle","loading","unloading","loaded","fault"};
    return (unsigned)s<sizeof(names)/sizeof(names[0])?names[s]:"invalid";
}
