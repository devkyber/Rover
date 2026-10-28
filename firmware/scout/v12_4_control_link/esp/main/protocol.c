#include "protocol.h"
#include <string.h>
bool scout_command_known(const char *name)
{
    static const char *const commands[]={"heartbeat","status","configure",
        "prepare","latch","launch","compress","decompress","compression_stop","stop","estop",
        "clear_fault","log_prepare","log_start","log_stop","log_export"};
    if (!name) return false;
    for (unsigned i=0;i<sizeof(commands)/sizeof(commands[0]);++i)
        if (!strcmp(name,commands[i])) return true;
    return false;
}
bool scout_json_bounded(const char *s,unsigned max_depth)
{
    unsigned depth=0;
    bool quoted=false,escaped=false;
    if (!s) return false;
    for (;*s;++s) {
        if (quoted) {
            if (escaped) escaped=false;
            else if (*s=='\\') escaped=true;
            else if (*s=='"') quoted=false;
        } else if (*s=='"') quoted=true;
        else if (*s=='{' || *s=='[') { if (++depth>max_depth) return false; }
        else if (*s=='}' || *s==']') { if (!depth) return false; --depth; }
    }
    return !quoted && !depth;
}
