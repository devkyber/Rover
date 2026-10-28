#ifndef SCOUT_CONTROL_H
#define SCOUT_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#define SCOUT_SERVO_PULL_US 2296u
#define SCOUT_SERVO_LOCK_US 1265u
#define SCOUT_SERVO_LAUNCH_US 1678u /* Nominal 110 degrees in the 544..2400 us mapping. */
typedef enum {
    CT_DISARMED, CT_LOADING, CT_UNLOADING, CT_LOADED, CT_FAULT
} scout_control_state_t;
typedef enum { SERVO_ROLE_OFF=0, SERVO_ROLE_PULL, SERVO_ROLE_LOCK, SERVO_ROLE_LAUNCH } scout_servo_role_t;
typedef struct {
    int motor_direction, encoder_sign;
    unsigned motor_percent, load_timeout_ms, stall_ms;
    unsigned hold_tolerance_counts, servo_neutral_us, servo_latch_us, latch_ms;
    unsigned release_ms; /* Legacy wire key, required to be zero. */
    unsigned servo_mode, servo_prepare_us, prepare_ms; /* Fixed position mode; timing fields are zero. */
} scout_mechanism_t;
typedef struct {
    scout_control_state_t state;
    scout_servo_role_t servo_role;
    scout_mechanism_t mechanism;
    bool configured, linked, needs_reset, boot_confirmation_required;
    /* Result is measured delta * encoder_sign, never a target or completion claim.
       Preserved through recovery/preparation; cleared only by the next load(). */
    bool compression_result_valid;
    unsigned last_servo_us;
    int64_t compression_result_counts;
    uint32_t cycle_id, launches_completed;
    int32_t origin_encoder, loaded_encoder;
    int64_t progress_counts, best_progress;
    uint64_t heartbeat_us, state_us, progress_us, neutral_until_us;
    const char *fault;
} scout_control_t;
typedef struct {
    int motor;
    unsigned motor_percent, loading_us;
    bool loading_enabled;
} scout_outputs_t;
void scout_control_init(scout_control_t *c);
bool scout_mechanism_valid(const scout_mechanism_t *m);
bool scout_control_configure(scout_control_t *c,const scout_mechanism_t *m);
void scout_control_heartbeat(scout_control_t *c,uint64_t now);
void scout_control_stop(scout_control_t *c,const char *reason,uint64_t now);
bool scout_control_clear_fault(scout_control_t *c,bool healthy,bool mechanism_reset);
bool scout_control_prepare(scout_control_t *c,uint64_t now,bool healthy);
bool scout_control_latch(scout_control_t *c,uint32_t cycle_id,uint64_t now,bool healthy);
bool scout_control_launch(scout_control_t *c,uint64_t now,bool healthy);
bool scout_control_load(scout_control_t *c,uint64_t now,int32_t encoder,bool healthy);
bool scout_control_unload(scout_control_t *c,uint64_t now,int32_t encoder,bool healthy);
bool scout_control_compression_stop(scout_control_t *c,uint64_t now,int32_t encoder);
scout_outputs_t scout_control_tick(scout_control_t *c,uint64_t now,int32_t encoder,bool healthy);
const char *scout_control_name(scout_control_state_t state);
bool scout_control_busy(const scout_control_t *c);
#endif
