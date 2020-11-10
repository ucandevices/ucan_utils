#include "log.h"
#include "ini.h"
#include "ucan_cfg.h"
#include "string.h"

#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
#define PARSE(s) (strcmp(value, s) == 0)

int ini_cfuc_handler(void *user, const char *section, const char *name,
                     const char *value)
{
    configuration *c = (configuration *)user;

    if (MATCH("caninit", "FrameFormat"))
    {
        if PARSE ("FDCAN_FRAME_CLASSIC")
            c->fdcanInitType.FrameFormat = FDCAN_FRAME_CLASSIC;
        else if PARSE ("FDCAN_FRAME_FD_NO_BRS")
            c->fdcanInitType.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
        else if PARSE ("FDCAN_FRAME_FD_BRS")
            c->fdcanInitType.FrameFormat = FDCAN_FRAME_FD_BRS;
        else
            log_error("Unknown FrameFormat");
    }
    else if(MATCH("caninit", "Mode"))
    {
        if PARSE ("FDCAN_MODE_INTERNAL_LOOPBACK")
            c->fdcanInitType.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
        else if PARSE ("FDCAN_MODE_NORMAL")
            c->fdcanInitType.Mode = FDCAN_MODE_NORMAL;
        else if PARSE ("FDCAN_MODE_RESTRICTED_OPERATION")
            c->fdcanInitType.Mode = FDCAN_MODE_RESTRICTED_OPERATION;
        else if PARSE ("FDCAN_MODE_BUS_MONITORING")
            c->fdcanInitType.Mode = FDCAN_MODE_BUS_MONITORING;
        else if PARSE ("FDCAN_MODE_EXTERNAL_LOOPBACK")
            c->fdcanInitType.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
        else
            log_error("Unknown FrameFormat");
    }
    else if (MATCH("caninit", "AutoRetransmission"))
    {
        c->fdcanInitType.AutoRetransmission = atoi(value);
    }
    else if (MATCH("caninit", "TransmitPause"))
    {
        c->fdcanInitType.TransmitPause = atoi(value);
    } else if (MATCH("caninit", "ProtocolException"))
    {
        c->fdcanInitType.ProtocolException = atoi(value);
    }
    else if (MATCH("caninit", "NominalPrescaler"))
    {
        c->fdcanInitType.NominalPrescaler = atoi(value);
    }
    else if (MATCH("caninit", "NominalSyncJumpWidth"))
    {
        c->fdcanInitType.NominalSyncJumpWidth = atoi(value);
    }
    else if (MATCH("caninit", "NominalTimeSeg1"))
    {
        c->fdcanInitType.NominalTimeSeg1 = atoi(value);
    }
    else if (MATCH("caninit", "NominalTimeSeg2"))
    {
        c->fdcanInitType.NominalTimeSeg2 = atoi(value);
    }
    else if (MATCH("caninit", "DataPrescaler"))
    {
        c->fdcanInitType.DataPrescaler = atoi(value);
    }
    else if (MATCH("caninit", "DataSyncJumpWidth"))
    {
        c->fdcanInitType.DataSyncJumpWidth = atoi(value);
    }
    else if (MATCH("caninit", "DataTimeSeg1"))
    {
        c->fdcanInitType.DataTimeSeg1 = atoi(value);
    }
    else if (MATCH("caninit", "DataTimeSeg2"))
    {
        c->fdcanInitType.DataTimeSeg2 = atoi(value);
    }
    else if (MATCH("caninit", "StdFiltersNbr"))
    {
        c->fdcanInitType.StdFiltersNbr = atoi(value);
    }
    else if (MATCH("caninit", "ExtFiltersNbr"))
    {
        c->fdcanInitType.ExtFiltersNbr = atoi(value);
    }
    else
    {
        log_error("Unknown Parameter %s", name);
        return 0; /* unknown section/name, error */
    }    
    return 1;
}

configuration config = {
    {.ClockDivider = 0,
     .FrameFormat = FDCAN_FRAME_FD_NO_BRS,
     .Mode = FDCAN_MODE_INTERNAL_LOOPBACK,
     .AutoRetransmission = DISABLE,
     .TransmitPause = DISABLE,
     .ProtocolException = DISABLE,
     .NominalPrescaler = 1,
     .NominalSyncJumpWidth = 1,
     .NominalTimeSeg1 = 13,
     .NominalTimeSeg2 = 2,
     .DataPrescaler = 1,
     .DataSyncJumpWidth = 1,
     .DataTimeSeg1 = 1,
     .DataTimeSeg2 = 1,
     .StdFiltersNbr = 0,
     .ExtFiltersNbr = 0,
     .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION}};

configuration *load_cfg(const char *cfg_path)
{

    if (ini_parse(cfg_path, ini_cfuc_handler, &config) < 0)
    {
        log_error("Can't load 'test.ini'\n");
        return NULL;
    }
    return &config;
}