#include "board_pins.h"
#include "actuators.h"
#include "control.h"
#include "capture.h"
#include "sensors.h"
#include "transport.h"
#include "protocol.h"
#include "settings.h"
#include "network.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "cJSON.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdatomic.h>
#include <string.h>

static portMUX_TYPE snapshot_lock=portMUX_INITIALIZER_UNLOCKED;
static scout_sensors_t snapshot;
static uint64_t snapshot_us;
static bool sensors_ready,actuators_ready;
static scout_control_t controller;
static uint32_t current_session,last_sequence;
typedef struct { unsigned operation,seconds; uint32_t session,seq; } log_request_t;
static QueueHandle_t log_requests;
static atomic_bool log_busy,motion_busy;
bool scout_motion_busy(void) { return atomic_load(&motion_busy); }
static scout_capture_status_t log_snapshot;
static uint32_t export_session;
static bool export_chunk(uint32_t offset,const void *data,size_t size)
{ return scout_transport_file_chunk(export_session,offset,data,size); }

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
static bool recent(uint64_t now,uint64_t stamp)
{ return stamp && (now<=stamp || now-stamp<250000); }
static scout_sensors_t read_snapshot(bool *healthy,uint64_t now)
{
    scout_sensors_t s;
    portENTER_CRITICAL(&snapshot_lock);
    s=snapshot;
    *healthy=sensors_ready && actuators_ready && snapshot_us &&
        (now<=snapshot_us || now-snapshot_us<250000);
    portEXIT_CRITICAL(&snapshot_lock);
    if (!recent(now,s.last_icm_us) || !recent(now,s.last_adxl_us) || !s.encoder_ready ||
        scout_actuators_fault() || scout_network_maintenance() || !scout_transport_data_ready()) *healthy=false;
    return s;
}
static bool actuator_link_ready(void)
{
    return actuators_ready && !scout_actuators_fault() &&
        !scout_network_maintenance();
}
static const char *servo_role_name(void)
{
    switch (controller.servo_role) {
    case SERVO_ROLE_PULL: return "pull";
    case SERVO_ROLE_LOCK: return "lock";
    case SERVO_ROLE_LAUNCH: return "launch";
    default: return "off";
    }
}
static bool dc_running(void)
{ return controller.state==CT_LOADING || controller.state==CT_UNLOADING; }
static const char *motor_action_name(void)
{ return controller.state==CT_LOADING?"compress":controller.state==CT_UNLOADING?"decompress":"stopped"; }
static void status(uint64_t now)
{
    bool healthy; scout_sensors_t s=read_snapshot(&healthy,now);
    portENTER_CRITICAL(&snapshot_lock);
    scout_capture_status_t log=log_snapshot;
    portEXIT_CRITICAL(&snapshot_lock);
    char text[SCOUT_JSON_MAX];
    snprintf(text,sizeof(text),"{\"event\":\"status\",\"api\":\"scout-process-v12\",\"session\":%lu,\"time_us\":%llu,"
        "\"firmware\":\"scout-process-v12.4-control-link-j5\",\"servo_count\":1,\"servo_connector\":\"J5\",\"launch_supported\":true,\"compression_stop_supported\":true,\"manual_latch_supported\":true,\"priority_stop_supported\":true,\"boot_confirmation_required\":%s,"
        "\"state\":\"%s\",\"fault\":\"%s\",\"configured\":%s,\"healthy\":%s,"
        "\"motor_duty_pct\":%u,\"load_timeout_ms\":%u,\"manual_compression_supported\":true,\"count_stop_enabled\":false,\"compression_result_valid\":%s,\"compression_result_counts\":%lld,\"compression_progress\":%lld,"
        "\"cycle_id\":%lu,\"motor_running\":%s,\"motor_action\":\"%s\",\"servo_role\":\"%s\","
        "\"icm_ok\":%s,\"adxl_ok\":%s,\"mag_ok\":%s,\"mag_id\":%u,\"mag_stream_supported\":true,\"mag_qualified\":false,\"mag_fresh\":%s,\"mag_age_ms\":%lld,\"driver_fault\":%s,"
        "\"encoder\":%d,\"encoder_ready\":%s,\"encoder_a\":%d,\"encoder_b\":%d,"
        "\"encoder_edges_a\":%lu,\"encoder_edges_b\":%lu,\"battery_mv\":%d,\"current_adc_mv\":%d,"
        "\"icm_hz\":%lu,\"adxl_hz\":%lu,\"icm_frames\":%lu,\"adxl_frames\":%lu,\"mag_frames\":%lu,"
        "\"icm_full\":%lu,\"adxl_full\":%lu,\"sensor_errors\":%lu,\"tx_dropped\":%lu,"
        "\"log_prepared\":%s,\"log_recording\":%s,\"log_records\":%lu,"
        "\"log_dropped\":%lu,\"log_errors\":%lu,\"log_busy\":%s}",
        (unsigned long)current_session,(unsigned long long)now,controller.boot_confirmation_required?"true":"false",
        scout_control_name(controller.state),controller.fault,controller.configured?"true":"false",healthy?"true":"false",
        controller.mechanism.motor_percent,controller.mechanism.load_timeout_ms,
        controller.compression_result_valid?"true":"false",(long long)controller.compression_result_counts,(long long)controller.progress_counts,
        (unsigned long)controller.cycle_id,dc_running()?"true":"false",motor_action_name(),servo_role_name(),
        s.icm_ok?"true":"false",s.adxl_ok?"true":"false",
        s.mag_ok?"true":"false",(unsigned)s.mag_id,recent(now,s.last_mag_us)?"true":"false",
        s.last_mag_us?(long long)(now>=s.last_mag_us?(now-s.last_mag_us)/1000:0):-1LL,scout_actuators_fault()?"true":"false",s.encoder_count,s.encoder_ready?"true":"false",
        s.encoder_a,s.encoder_b,(unsigned long)s.encoder_edges_a,(unsigned long)s.encoder_edges_b,
        s.battery_mv,s.current_adc_mv,(unsigned long)s.icm_hz,(unsigned long)s.adxl_hz,
        (unsigned long)s.icm_frames,(unsigned long)s.adxl_frames,(unsigned long)s.mag_frames,
        (unsigned long)s.icm_full,(unsigned long)s.adxl_full,(unsigned long)s.io_errors,(unsigned long)scout_transport_dropped(),
        log.prepared?"true":"false",log.recording?"true":"false",(unsigned long)log.records,
        (unsigned long)log.dropped,(unsigned long)log.write_errors,atomic_load(&log_busy)?"true":"false");
    scout_transport_json(text);
}
static void reply(uint32_t seq,bool ok,const char *error)
{
    char text[420];
    snprintf(text,sizeof(text),"{\"event\":\"ack\",\"api\":\"scout-process-v12\",\"session\":%lu,\"seq\":%lu,\"ok\":%s,\"error\":\"%s\",\"cycle_id\":%lu,\"time_us\":%llu,\"state\":\"%s\",\"motor_running\":%s,\"motor_action\":\"%s\",\"servo_role\":\"%s\"}",
        (unsigned long)current_session,(unsigned long)seq,ok?"true":"false",error?error:"none",(unsigned long)controller.cycle_id,(unsigned long long)esp_timer_get_time(),scout_control_name(controller.state),dc_running()?"true":"false",motor_action_name(),servo_role_name());
    scout_transport_json(text);
}
static void handle(const scout_command_t *message,uint64_t now,bool healthy,int encoder)
{
    cJSON *root=scout_json_bounded(message->json,8)?cJSON_ParseWithOpts(message->json,NULL,true):NULL;
    int64_t token=0,seq=0;
    const cJSON *cmd=cJSON_GetObjectItemCaseSensitive(root,"cmd");
    if (!cJSON_IsObject(root) || !cJSON_IsString(cmd) ||
        !integer(root,"session",1,UINT32_MAX,&token) || !integer(root,"seq",1,UINT32_MAX,&seq) ||
        (uint32_t)token!=current_session || message->session!=current_session || current_session!=scout_transport_session()) {
        reply(0,false,"invalid_envelope_or_session"); cJSON_Delete(root); return;
    }
    if ((uint32_t)seq<=last_sequence) {
        reply((uint32_t)seq,false,"duplicate_or_old_sequence"); cJSON_Delete(root); return;
    }
    last_sequence=(uint32_t)seq;
    if (cJSON_GetObjectItemCaseSensitive(root,"angle_deg") || cJSON_GetObjectItemCaseSensitive(root,"duration_ms")) {
        reply((uint32_t)seq,false,"angle_commands_removed"); cJSON_Delete(root); return;
    }
    if (cJSON_GetObjectItemCaseSensitive(root,"counts")) {
        reply((uint32_t)seq,false,"count_target_commands_removed"); cJSON_Delete(root); return;
    }
    const char *name=cmd->valuestring;
    bool known=scout_command_known(name);
    bool ok=false;
    if (known) scout_control_heartbeat(&controller,now);
    /* Telemetry never authorizes or cancels independent manual output commands. */
    bool motion_ok=healthy;
    if (!strcmp(name,"heartbeat")) ok=true;
    else if (!strcmp(name,"status")) { status(now); ok=true; }
    else if (!strcmp(name,"configure")) {
        scout_mechanism_t m;
        ok=!atomic_load(&log_busy) && scout_mechanism_from_json(cJSON_GetObjectItemCaseSensitive(root,"mechanism"),&m) &&
            scout_control_configure(&controller,&m);
    } else if (!strcmp(name,"prepare")) ok=scout_control_prepare(&controller,now,motion_ok);
    else if (!strcmp(name,"latch")) ok=scout_control_latch(&controller,0,now,motion_ok);
    else if (!strcmp(name,"launch")) ok=scout_control_launch(&controller,now,motion_ok);
    else if (!strcmp(name,"compress")) ok=scout_control_load(&controller,now,encoder,motion_ok);
    else if (!strcmp(name,"decompress")) ok=scout_control_unload(&controller,now,encoder,motion_ok);
    else if (!strcmp(name,"compression_stop")) { ok=scout_control_compression_stop(&controller,now,encoder); }
    else if (!strcmp(name,"stop")) { scout_control_stop(&controller,"operator_stop",now); ok=true; }
    else if (!strcmp(name,"estop")) { scout_control_stop(&controller,"estop",now); ok=true; }
    else if (!strcmp(name,"clear_fault")) {
        bool reset=cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(root,"mechanism_reset"));
        ok=scout_control_clear_fault(&controller,controller.boot_confirmation_required || motion_ok,reset);
    } else if (!strncmp(name,"log_",4)) {
        log_request_t request={.session=current_session,.seq=(uint32_t)seq};
        if (!strcmp(name,"log_prepare")) request.operation=1;
        else if (!strcmp(name,"log_start")) {
            int64_t seconds;
            if (integer(root,"seconds",1,10,&seconds)) { request.operation=2; request.seconds=(unsigned)seconds; }
        } else if (!strcmp(name,"log_stop")) request.operation=3;
        else if (!strcmp(name,"log_export")) request.operation=4;
        bool stopped=(controller.state==CT_DISARMED || controller.state==CT_FAULT) && !controller.neutral_until_us;
        bool permitted=request.operation==3 || !scout_control_busy(&controller) || (request.operation==4 && stopped);
        if (request.operation && permitted && !atomic_exchange(&log_busy,true)) {
            ok=xQueueSend(log_requests,&request,0)==pdTRUE;
            if (!ok) atomic_store(&log_busy,false);
        }
    }
    reply((uint32_t)seq,ok,ok?NULL:known?(!motion_ok?"actuator_or_link_unavailable":"invalid_parameters_or_state"):"unknown_command");
    cJSON_Delete(root);
}
/* Stop requests bypass the ordinary queue and cancel its pending actions. */
static bool handle_urgent(uint64_t now,bool healthy,int encoder)
{
    scout_command_t command; uint32_t barrier=0;
    if (!scout_transport_urgent(current_session,last_sequence,&command,&barrier)) return false;
    handle(&command,now,healthy,encoder);
    if (barrier>last_sequence) last_sequence=barrier;
    return true;
}
static void sample(uint8_t source,uint8_t flags,uint64_t time,const void *data,size_t size)
{
    scout_capture_sample(source,flags,time,data,size);
    scout_transport_sample(source,flags,time,data,size);
}
static void acquisition_task(void *arg)
{
    (void)arg;
    scout_sensors_t s={0}; esp_err_t rc=scout_sensors_init(&s);
#ifdef CONFIG_SCOUT_SHOCK_PROFILE
    if (rc==ESP_OK) rc=scout_sensors_profile(&s,true);
#endif
    esp_err_t storage=scout_capture_init();
    for (;;) {
        log_request_t request;
        if (xQueueReceive(log_requests,&request,0)==pdTRUE) {
            esp_err_t result=storage;
            if (storage==ESP_OK) {
                if (request.operation==1) result=scout_capture_prepare();
                else if (request.operation==2 && rc==ESP_OK) {
                    scout_header_t h={.magic={'S','C','O','U','T','0','0','1'},.version=1,
                        .record_size=40,.icm_hz=s.icm_hz,.adxl_hz=s.adxl_hz,.mag_hz=s.mag_ok?50:0,
                        .duration_ms=request.seconds*1000,.start_us=(uint64_t)esp_timer_get_time(),
                        .icm_id=s.icm_id,.adxl_id=s.adxl_id,.mag_id=s.mag_id,
                        .profile=s.icm_hz>400?1:0};
                    h.crc32=scout_crc32(&h,offsetof(scout_header_t,crc32));
                    result=scout_capture_start(&h);
                } else if (request.operation==3) result=scout_capture_stop();
                else if (request.operation==4 && request.session==scout_transport_session()) {
                    export_session=request.session;
                    result=scout_capture_export(export_chunk);
                }
                else result=ESP_ERR_INVALID_STATE;
            }
            char text[300];
            snprintf(text,sizeof(text),"{\"event\":\"log_result\",\"session\":%lu,\"seq\":%lu,\"ok\":%s,\"error\":\"%s\"}",
                (unsigned long)request.session,(unsigned long)request.seq,result==ESP_OK?"true":"false",esp_err_to_name(result));
            if (request.session==scout_transport_session()) scout_transport_json(text);
            atomic_store(&log_busy,false);
        }
        if (rc==ESP_OK) scout_sensors_poll(&s,sample);
        scout_capture_status_t log=scout_capture_status();
        if (log.recording) {
            scout_capture_service();
            if ((uint64_t)esp_timer_get_time()>=log.stop_us) scout_capture_stop();
        }
        portENTER_CRITICAL(&snapshot_lock);
        snapshot=s; snapshot_us=(uint64_t)esp_timer_get_time(); sensors_ready=rc==ESP_OK;
        log_snapshot=scout_capture_status();
        portEXIT_CRITICAL(&snapshot_lock);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
static void control_task(void *arg)
{
    (void)arg; TickType_t tick=xTaskGetTickCount(); uint64_t next_status=0;
    if (esp_task_wdt_add(NULL)!=ESP_OK) { scout_actuators_park(); vTaskDelete(NULL); return; }
    scout_control_init(&controller);
    for (;;) {
        uint64_t now=(uint64_t)esp_timer_get_time();
        uint32_t session=scout_transport_session();
        if (session!=current_session) {
            if (scout_control_busy(&controller))
                scout_control_stop(&controller,"connection_changed",now);
            current_session=session; last_sequence=0; controller.linked=false;
        }
        bool observation_ok; scout_sensors_t s=read_snapshot(&observation_ok,now);
        int encoder=s.encoder_count;
        (void)scout_encoder_read(&encoder);
        bool healthy=actuator_link_ready();
        scout_command_t command;
        for (unsigned i=0;i<8;++i) {
            if (handle_urgent(now,healthy,encoder)) break;
            if (!scout_transport_command(&command)) break;
            handle(&command,now,healthy,encoder);
        }
        handle_urgent(now,healthy,encoder);
        scout_outputs_t out=scout_control_tick(&controller,now,encoder,healthy);
        atomic_store(&motion_busy,scout_control_busy(&controller));
        if (actuators_ready && scout_actuators_apply(&out,now)!=ESP_OK)
        { scout_control_stop(&controller,"output_error",now); }
        if (current_session && now>=next_status) { status(now); next_status=now+100000; }
        esp_task_wdt_reset();
        vTaskDelayUntil(&tick,pdMS_TO_TICKS(10));
    }
}
void app_main(void)
{
    actuators_ready=scout_actuators_init()==ESP_OK;
    if (!actuators_ready) scout_actuators_park();
    esp_task_wdt_config_t watchdog={.timeout_ms=2000,.idle_core_mask=3,.trigger_panic=true};
    if (esp_task_wdt_reconfigure(&watchdog)!=ESP_OK) { scout_actuators_park(); return; }
    /* Motor outputs are parked before network or sensor initialization. */
    if (scout_transport_init()!=ESP_OK) { scout_actuators_park(); return; }
    log_requests=xQueueCreate(1,sizeof(log_request_t));
    if (!log_requests) { scout_actuators_park(); return; }
    if (xTaskCreatePinnedToCore(acquisition_task,"imu_acq",8192,NULL,8,NULL,1)!=pdPASS ||
        xTaskCreatePinnedToCore(control_task,"local_control",12288,NULL,10,NULL,0)!=pdPASS) {
        scout_actuators_park(); esp_restart();
    }
}
