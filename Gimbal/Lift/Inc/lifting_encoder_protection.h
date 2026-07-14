#ifndef LIFTING_ENCODER_PROTECTION_H
#define LIFTING_ENCODER_PROTECTION_H

#include "lifting_types.h"

#if ROBOT_SELECT == NEW
void LiftEncoderProtection_Init(void);
void LiftEncoderProtection_BeginMotion(void);
LiftFault LiftEncoderProtection_AfterPid(bool detection_enabled);
#endif

#endif
