/**
 * @file cell_balancing_mgr.h
 * @brief Cell Balancing Manager - Application Layer (TBD)
 */

#ifndef APP_CELL_BALANCING_MGR_H
#define APP_CELL_BALANCING_MGR_H

#include <stdint.h>
#include <stdbool.h>

void CellBalance_Init(void);
void CellBalance_Update(void);
bool CellBalance_IsBalancing(void);

#endif