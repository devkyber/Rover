#include "network.h"
#include "transport.h"
#include "protocol.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "cJSON.h"
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

static atomic_bool maintenance;
bool scout_network_maintenance(void) { return atomic_load(&maintenance); }
#ifdef CONFIG_SCOUT_LINK_WIFI
static esp_netif_t *netif;
static bool credentials;
static atomic_uint last_disconnect_reason, disconnect_count;
static void event(void *arg,esp_event_base_t base,int32_t id,void *data)
{
    (void)arg;
    if (base==WIFI_EVENT && id==WIFI_EVENT_STA_DISCONNECTED && data) {
        const wifi_event_sta_disconnected_t *lost=data;
        atomic_store(&last_disconnect_reason,lost->reason);
        atomic_fetch_add(&disconnect_count,1);
    }
    if (base==WIFI_EVENT && credentials &&
        (id==WIFI_EVENT_STA_START || id==WIFI_EVENT_STA_DISCONNECTED)) esp_wifi_connect();
}
static void usb_reply(const char *error)
{
    esp_netif_ip_info_t info={0}; char ip[16]="0.0.0.0",text[512];
    wifi_ap_record_t ap; bool connected=esp_wifi_sta_get_ap_info(&ap)==ESP_OK;
    if (netif && esp_netif_get_ip_info(netif,&info)==ESP_OK)
        inet_ntop(AF_INET,&info.ip.addr,ip,sizeof(ip));
    snprintf(text,sizeof(text),"{\"event\":\"wifi\",\"api\":\"scout-process-v12\",\"firmware\":\"scout-process-v12.4-control-link-j5\",\"ok\":%s,\"error\":\"%s\",\"ip\":\"%s\",\"configured\":%s,\"connected\":%s,\"port\":9000,\"data_port\":9001,\"last_disconnect_reason\":%u,\"disconnects\":%u,\"uptime_ms\":%llu,\"reset_reason\":%d}\n",
        error?"false":"true",error?error:"none",ip,credentials?"true":"false",connected?"true":"false",
        atomic_load(&last_disconnect_reason),atomic_load(&disconnect_count),
        (unsigned long long)(esp_timer_get_time()/1000),(int)esp_reset_reason());
    usb_serial_jtag_write_bytes(text,strlen(text),pdMS_TO_TICKS(100));
}
static const char *field(const cJSON *root,const char *name)
{
    const cJSON *v=cJSON_GetObjectItemCaseSensitive(root,name);
    return cJSON_IsString(v)?v->valuestring:NULL;
}
static bool static_ip_valid(const char *ip,const char *gateway,const char *mask,esp_netif_ip_info_t *info)
{
    if (!ip || !gateway || !mask || inet_pton(AF_INET,ip,&info->ip.addr)!=1 ||
        inet_pton(AF_INET,gateway,&info->gw.addr)!=1 || inet_pton(AF_INET,mask,&info->netmask.addr)!=1) return false;
    uint32_t m=ntohl(info->netmask.addr),host=ntohl(info->ip.addr)&~m;
    return m && (~m&(~m+1u))==0 && host && host!=~m &&
        (info->ip.addr&info->netmask.addr)==(info->gw.addr&info->netmask.addr);
}
static const char *auth_name(wifi_auth_mode_t mode)
{
    switch (mode) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
    default: return "OTHER";
    }
}
static void scan_reply(const char *ssid)
{
    /* Read-only scan of the requested SSID; never changes stored credentials. */
    if (!ssid || !strlen(ssid) || strlen(ssid)>32) { usb_reply("invalid_scan_ssid"); return; }
    if (scout_motion_busy() || scout_transport_session()) { usb_reply("disconnect_jetson_and_disarm_first"); return; }
    wifi_scan_config_t config={.ssid=(uint8_t *)ssid,.show_hidden=true};
    esp_err_t rc=esp_wifi_scan_start(&config,true);
    wifi_ap_record_t aps[8]; uint16_t count=8;
    if (rc==ESP_OK) rc=esp_wifi_scan_get_ap_records(&count,aps);
    cJSON *out=cJSON_CreateObject();
    if (!out) { usb_reply("scan_no_memory"); return; }
    cJSON_AddStringToObject(out,"event","wifi");
    cJSON_AddBoolToObject(out,"ok",rc==ESP_OK);
    cJSON_AddBoolToObject(out,"scan",true);
    cJSON_AddStringToObject(out,"ssid",ssid);
    cJSON_AddBoolToObject(out,"found",rc==ESP_OK && count>0);
    cJSON_AddStringToObject(out,"error",rc==ESP_OK?"none":esp_err_to_name(rc));
    if (rc==ESP_OK && count) {
        cJSON_AddStringToObject(out,"authmode",auth_name(aps[0].authmode));
        cJSON_AddNumberToObject(out,"channel",aps[0].primary);
        cJSON_AddNumberToObject(out,"rssi",aps[0].rssi);
    }
    char *text=cJSON_PrintUnformatted(out);
    if (text) { usb_serial_jtag_write_bytes(text,strlen(text),pdMS_TO_TICKS(100));
        usb_serial_jtag_write_bytes("\n",1,pdMS_TO_TICKS(100)); cJSON_free(text); }
    cJSON_Delete(out);
}
static void provision(const char *text)
{
    cJSON *root=scout_json_bounded(text,4)?cJSON_ParseWithOpts(text,NULL,true):NULL;
    const char *cmd=field(root,"cmd");
    if (cmd && !strcmp(cmd,"wifi_status")) { usb_reply(NULL); cJSON_Delete(root); return; }
    if (cmd && !strcmp(cmd,"wifi_scan")) { scan_reply(field(root,"ssid")); cJSON_Delete(root); return; }
    if (!cmd || strcmp(cmd,"wifi_config")) { usb_reply("invalid_command"); cJSON_Delete(root); return; }
    const char *ssid=field(root,"ssid"),*password=field(root,"password"),*ip=field(root,"ip");
    const char *gateway=field(root,"gateway"),*mask=field(root,"netmask");
    esp_netif_ip_info_t info={0};
    if (!ssid || !password || !strlen(ssid) || strlen(ssid)>32 || strlen(password)<8 || strlen(password)>63 ||
        (ip && !static_ip_valid(ip,gateway,mask,&info))) {
        usb_reply("invalid_credentials_or_static_ip"); cJSON_Delete(root); return;
    }
    if (scout_motion_busy() || scout_transport_session()) { usb_reply("disconnect_jetson_and_disarm_first"); cJSON_Delete(root); return; }
    atomic_store(&maintenance,true);
    nvs_handle_t handle;
    esp_err_t rc=nvs_open("scout_wifi",NVS_READWRITE,&handle);
    if (rc==ESP_OK) {
        rc=nvs_set_str(handle,"ssid",ssid);
        if (rc==ESP_OK) rc=nvs_set_str(handle,"password",password);
        if (rc==ESP_OK) rc=nvs_set_str(handle,"ip",ip?ip:"");
        if (rc==ESP_OK) rc=nvs_set_str(handle,"gateway",gateway?gateway:"");
        if (rc==ESP_OK) rc=nvs_set_str(handle,"netmask",mask?mask:"");
        if (rc==ESP_OK) rc=nvs_commit(handle);
        nvs_close(handle);
    }
    cJSON_Delete(root);
    usb_reply(rc==ESP_OK?NULL:"nvs_write_failed");
    if (rc==ESP_OK) {
        usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(200));
        esp_restart();
    }
    atomic_store(&maintenance,false);
}
static void provisioning_task(void *arg)
{
    (void)arg; char line[512]; size_t used=0; bool overflow=false;
    for (;;) {
        uint8_t bytes[128]; int n=usb_serial_jtag_read_bytes(bytes,sizeof(bytes),pdMS_TO_TICKS(20));
        for (int i=0;i<n;++i) {
            if (bytes[i]=='\n' || bytes[i]=='\r') {
                if (used && !overflow) { line[used]=0; provision(line); }
                else if (overflow) usb_reply("command_too_long");
                used=0; overflow=false;
            } else if (!bytes[i]) overflow=true;
            else if (used<sizeof(line)-1) line[used++]=(char)bytes[i];
            else overflow=true;
        }
    }
}
esp_err_t scout_network_init(void)
{
    esp_err_t rc=nvs_flash_init(); if (rc!=ESP_OK) return rc;
    if ((rc=esp_netif_init())!=ESP_OK) return rc;
    if ((rc=esp_event_loop_create_default())!=ESP_OK) return rc;
    netif=esp_netif_create_default_wifi_sta(); if (!netif) return ESP_FAIL;
    wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();
    if ((rc=esp_wifi_init(&init))!=ESP_OK) return rc;
    if ((rc=esp_wifi_set_storage(WIFI_STORAGE_RAM))!=ESP_OK) return rc;
    if ((rc=esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,event,NULL))!=ESP_OK) return rc;
    wifi_config_t config={0}; nvs_handle_t handle;
    char ssid[33]="",password[64]="",ip[16]="",gateway[16]="",mask[16]="";
    if (nvs_open("scout_wifi",NVS_READONLY,&handle)==ESP_OK) {
        size_t n=sizeof(ssid); nvs_get_str(handle,"ssid",ssid,&n);
        n=sizeof(password); nvs_get_str(handle,"password",password,&n);
        n=sizeof(ip); nvs_get_str(handle,"ip",ip,&n);
        n=sizeof(gateway); nvs_get_str(handle,"gateway",gateway,&n);
        n=sizeof(mask); nvs_get_str(handle,"netmask",mask,&n); nvs_close(handle);
    }
    esp_netif_ip_info_t address={0};
    if (ip[0] && static_ip_valid(ip,gateway,mask,&address)) {
        if ((rc=esp_netif_dhcpc_stop(netif))!=ESP_OK) return rc;
        if ((rc=esp_netif_set_ip_info(netif,&address))!=ESP_OK) return rc;
    }
    credentials=strlen(ssid)>0 && strlen(password)>=8;
    memcpy(config.sta.ssid,ssid,strlen(ssid)); memcpy(config.sta.password,password,strlen(password));
    config.sta.threshold.authmode=WIFI_AUTH_WPA2_PSK;
    if ((rc=esp_wifi_set_mode(WIFI_MODE_STA))!=ESP_OK) return rc;
    if (credentials && (rc=esp_wifi_set_config(WIFI_IF_STA,&config))!=ESP_OK) return rc;
    if ((rc=esp_wifi_start())!=ESP_OK) return rc;
    esp_wifi_set_ps(WIFI_PS_NONE);
    usb_serial_jtag_driver_config_t usb={.tx_buffer_size=1024,.rx_buffer_size=1024};
    if ((rc=usb_serial_jtag_driver_install(&usb))!=ESP_OK) return rc;
    return xTaskCreatePinnedToCore(provisioning_task,"wifi_setup",6144,NULL,3,NULL,0)==pdPASS?ESP_OK:ESP_ERR_NO_MEM;
}
#else
esp_err_t scout_network_init(void) { return ESP_OK; }
#endif
