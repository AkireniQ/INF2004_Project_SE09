#include "subsystem2.h"
#include "motion_config.h"
#include "motion_transport.h"
#include "motion_hw.h"
EXPORT ER subsystem2_init(void)
{
    s2_hw_safe_outputs();
    return s2_transport_init();
}
EXPORT ER subsystem2_start(void)
{
    /* TODO TEAM: keep the owner pinned to TP_PRC2 unless IRQ routing is changed too. */
    ID id=app_start_task((FP)motion_control_task,S2_MOTION_PRIORITY,S2_TASK_AFFINITY,"motion_control");
    return id<E_OK?(ER)id:E_OK;
}
