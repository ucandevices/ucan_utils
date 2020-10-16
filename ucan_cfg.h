#ifndef _CFUC_CFG_H
#define _CFUC_CFG_H

#include "./ucan_fd_protocol_stm32g431.h"


typedef struct
{    
    FDCAN_InitTypeDef fdcanInitType;

} configuration;


extern configuration config;
configuration* load_cfg(const char* cfg_path);




#endif