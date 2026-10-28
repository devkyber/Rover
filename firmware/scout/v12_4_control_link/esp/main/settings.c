#include "settings.h"
#include <math.h>
#include <string.h>
static bool number(const cJSON *o,const char *key,double low,double high,double *out)
{
    const cJSON *v=cJSON_GetObjectItemCaseSensitive(o,key);
    if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble) || v->valuedouble<low || v->valuedouble>high) return false;
    *out=v->valuedouble; return true;
}
static bool integer(const cJSON *o,const char *key,double low,double high,int64_t *out)
{
    double v;
    if (!number(o,key,low,high,&v) || trunc(v)!=v) return false;
    *out=(int64_t)v; return true;
}
bool scout_mechanism_from_json(const cJSON *o,scout_mechanism_t *m)
{
    int64_t v; memset(m,0,sizeof(*m));
    static const char *const keys[]={"servo_model","servo_mode","motor_direction","encoder_sign",
        "motor_duty_pct","load_timeout_ms","stall_ms",
        "hold_tolerance_counts","servo_neutral_us","prepare_ms","latch_ms","release_ms"};
    const cJSON *entry=NULL;
    cJSON_ArrayForEach(entry,o) {
        bool allowed=false;
        for (unsigned i=0;i<sizeof(keys)/sizeof(keys[0]);++i)
            if (entry->string && !strcmp(entry->string,keys[i])) { allowed=true; break; }
        if (!allowed) return false;
    }
    /* Physical positions belong to the PCB, never to a Jetson command payload. */
    m->servo_prepare_us=SCOUT_SERVO_PULL_US;
    m->servo_latch_us=SCOUT_SERVO_LOCK_US;
    const cJSON *model=cJSON_GetObjectItemCaseSensitive(o,"servo_model");
    if (!cJSON_IsObject(o) || !cJSON_IsString(model) || !model->valuestring[0]) return false;
    const cJSON *mode=cJSON_GetObjectItemCaseSensitive(o,"servo_mode");
    if (!cJSON_IsString(mode)) return false;
    m->servo_mode=!strcmp(mode->valuestring,"position")?1:0;
#define PARAM(key,lo,hi,member) do { if (!integer(o,key,lo,hi,&v)) return false; m->member=v; } while (0)
    PARAM("motor_direction",-1,1,motor_direction);
    PARAM("encoder_sign",-1,1,encoder_sign);
    PARAM("motor_duty_pct",1,100,motor_percent);
    PARAM("load_timeout_ms",0,0,load_timeout_ms);
    PARAM("stall_ms",0,0,stall_ms);
    PARAM("hold_tolerance_counts",0,0,hold_tolerance_counts);
    PARAM("servo_neutral_us",0,0,servo_neutral_us);
    PARAM("prepare_ms",0,0,prepare_ms);
    PARAM("latch_ms",0,0,latch_ms);
    PARAM("release_ms",0,0,release_ms);
#undef PARAM
    return scout_mechanism_valid(m);
}
