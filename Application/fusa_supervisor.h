/**
 * @file fusa_supervisor.h
 * @brief Functional Safety Supervisor - Application Layer
 */

#ifndef APP_FUSA_SUPERVISOR_H
#define APP_FUSA_SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>

void FuSa_Init(void);
bool FuSa_PerformSafetyChecks(void);
bool FuSa_GetSafetyStatus(void);

#endif