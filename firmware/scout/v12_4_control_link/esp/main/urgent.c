#include "urgent.h"
#include "protocol.h"
#include "cJSON.h"
#include <math.h>
#include <string.h>

static bool uint32_field(const cJSON *root,const char *key,uint32_t *out)
{
    const cJSON *v=cJSON_GetObjectItemCaseSensitive(root,key);
    if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble) || v->valuedouble<1 ||
        v->valuedouble>UINT32_MAX || floor(v->valuedouble)!=v->valuedouble) return false;
    *out=(uint32_t)v->valuedouble; return true;
}
const char *scout_urgent_name(scout_urgent_kind_t kind)
{
    return kind==URGENT_ESTOP?"estop":kind==URGENT_STOP?"stop":kind==URGENT_COMPRESSION_STOP?"compression_stop":"";
}
bool scout_urgent_parse(const char *json,uint32_t session,scout_urgent_item_t *item)
{
    if (!session || !json || !item || !scout_json_bounded(json,8)) return false;
    cJSON *root=cJSON_ParseWithOpts(json,NULL,true);
    const cJSON *name=cJSON_GetObjectItemCaseSensitive(root,"cmd");
    scout_urgent_item_t v={0};
    if (cJSON_IsString(name)) {
        if (!strcmp(name->valuestring,"estop")) v.kind=URGENT_ESTOP;
        else if (!strcmp(name->valuestring,"stop")) v.kind=URGENT_STOP;
        else if (!strcmp(name->valuestring,"compression_stop")) v.kind=URGENT_COMPRESSION_STOP;
    }
    bool okay=cJSON_IsObject(root) && v.kind && uint32_field(root,"session",&v.session) &&
        v.session==session && uint32_field(root,"seq",&v.sequence);
    cJSON_Delete(root);
    if (okay) { v.barrier=v.sequence; *item=v; }
    return okay;
}
void scout_urgent_reset(scout_urgent_t *inbox) { memset(inbox,0,sizeof(*inbox)); }
void scout_urgent_offer(scout_urgent_t *inbox,const scout_urgent_item_t *item)
{
    if (item->kind<URGENT_COMPRESSION_STOP || item->kind>URGENT_ESTOP || !item->session || !item->sequence) return;
    scout_urgent_item_t *slot=&inbox->slots[item->kind-1];
    if (!slot->sequence || slot->session!=item->session || item->sequence>slot->sequence) *slot=*item;
}
bool scout_urgent_take(scout_urgent_t *inbox,uint32_t session,uint32_t last_sequence,scout_urgent_item_t *item)
{
    scout_urgent_item_t selected={0}; uint32_t barrier=last_sequence;
    for (unsigned i=0;i<3;++i) {
        scout_urgent_item_t v=inbox->slots[i];
        if (v.session!=session || v.sequence<=last_sequence) continue;
        if (v.sequence>barrier) barrier=v.sequence;
        if (v.kind>selected.kind) selected=v;
    }
    scout_urgent_reset(inbox);
    if (!selected.kind) return false;
    selected.barrier=barrier; *item=selected; return true;
}
