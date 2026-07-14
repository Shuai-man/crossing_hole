#ifndef LIFTING_TOF_PROTECTION_H
#define LIFTING_TOF_PROTECTION_H

#include "lifting_types.h"

#if ROBOT_SELECT == OLD
void LiftTofProtection_Init(void);
void LiftTofProtection_BeginMotion(void);
LiftFault LiftTofProtection_BeforePid(bool detection_enabled);
LiftFault LiftTofProtection_AfterPid(void);
bool LiftTofProtection_OcclusionRecovered(void);
#endif

#endif
