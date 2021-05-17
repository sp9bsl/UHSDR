/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */
/************************************************************************************
 **                                                                                 **
 **                                        UHSDR                                    **
 **               a powerful firmware for STM32 based SDR transceivers              **
 **                                                                                 **
 **---------------------------------------------------------------------------------**
 **                                                                                 **
 **  Description:                                                                   **
 **  Licence:       GNU GPLv3                                                      **
 ************************************************************************************/

// Common
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "radio_management.h"
#include "rfboard_interface.h"
#include "profiling.h"


#include "osc_interface.h"
#include "osc_ducddc_df8oe.h"
#include "osc_SParkle.h"
#include "mchf_rfboard.h"


//specyfic hardware features structure
__MCHF_SPECIALMEM HardwareRFBoard RFboard;

// this structure MUST match the order of entries in power_level_t !
static const power_level_desc_t mchf_rf_power_levels[] =
{
        { .id = PA_LEVEL_FULL,   .mW = 0,    }, // we use 0 to indicate max power
        { .id = PA_LEVEL_HIGH,   .mW = 5000, },
        { .id = PA_LEVEL_MEDIUM, .mW = 2000, },
        { .id = PA_LEVEL_LOW,    .mW = 1000, },
        { .id = PA_LEVEL_MINIMAL,.mW =  500, },
};


const pa_power_levels_info_t mchf_power_levelsInfo =
{
        .levels = mchf_rf_power_levels,
        .count = sizeof(mchf_rf_power_levels)/sizeof(*mchf_rf_power_levels),
        .power_factor = 1.0f,
};

const pa_power_levels_info_t mchf_ddcduc_power_levelsInfo =
{
        .levels = mchf_rf_power_levels,
        .count = sizeof(mchf_rf_power_levels)/sizeof(*mchf_rf_power_levels),
        .power_factor = 2.0f,
};

#ifdef RF_BRD_LAPWING
const pa_info_t mchf_pa =
{
        .name  = "Lapwing PA",
        .reference_power = 5000.0,
        .max_freq = 1300 * 1000000,
        .min_freq = 1240 * 1000000,
        .max_am_power = 2000,
        .max_power = 20000,
};
#else
const pa_info_t mchf_pa =
{
        .name  = "mcHF PA",
        .reference_power = 5000.0,
        .max_freq = 32000000,
        .min_freq =  1800000,
        .max_am_power = 2000,
        .max_power = 10000,
};
#endif // LAPWING

#ifdef USE_OSC_SParkle
// this structure MUST match the order of entries in power_level_t !
static const power_level_desc_t SParkle_rf_power_levels[] =
{
        { .id = PA_LEVEL_FULL,   .mW = 0,    }, // we use 0 to indicate max power
        { .id = PA_LEVEL_HIGH,   .mW = 20000, },
        { .id = PA_LEVEL_MEDIUM, .mW = 10000, },
        { .id = PA_LEVEL_LOW,    .mW = 5000, },
        { .id = PA_LEVEL_MINIMAL,.mW = 1000, },
};

const pa_power_levels_info_t SParkle_power_levelsInfo =
{
        .levels = SParkle_rf_power_levels,
        .count = sizeof(SParkle_rf_power_levels)/sizeof(*SParkle_rf_power_levels),
        .power_factor = TX_POWER_FACTOR_MAX_DUC_INTERNAL,
};

const pa_info_t SParkle_pa =
{
        .name  = "SParkle PA",
        .reference_power = 20000.0,
        .max_freq = 150000000,
        .min_freq =  1800000,
        .max_am_power = 25000,
        .max_power = 50000,
};
#endif

#ifdef USE_OSC_DUCDDC
// this structure MUST match the order of entries in power_level_t !
static const power_level_desc_t Df8oe_DdcDuc_rf_power_levels[] =
{
        { .id = PA_LEVEL_FULL,   .mW = 0,    }, // we use 0 to indicate max power
        { .id = PA_LEVEL_HIGH,   .mW = 10000, },
        { .id = PA_LEVEL_MEDIUM, .mW = 5000, },
        { .id = PA_LEVEL_LOW,    .mW = 2000, },
        { .id = PA_LEVEL_MINIMAL,.mW = 1000, },
};

const pa_power_levels_info_t Df8oe_DdcDuc_power_levelsInfo =
{
        .levels = Df8oe_DdcDuc_rf_power_levels,
        .count = sizeof(Df8oe_DdcDuc_rf_power_levels)/sizeof(*Df8oe_DdcDuc_rf_power_levels),
        .power_factor = 2.0f,
};

const pa_info_t Df8oe_DdcDuc_pa =
{
        .name  = "Df8oe_DdcDuc PA",
        .reference_power = 10000.0,
        .max_freq = 76000000,
        .min_freq =  1800000,
        .max_am_power = 25000,
        .max_power = 50000,
};
#endif




bool RFBoard_Dummy_PrepareTx(void)
{
    return true;
}

bool RFBoard_Dummy_PrepareRx(void)
{
    return true;
}

bool RFBoard_Dummy_EnableRx(void)
{
    return true;
}

bool RFBoard_Dummy_EnableTx(void)
{
    return true;
}

bool RFBoard_Dummy_ChangeFrequency(uint32_t frequency)
{
    return true;
}

bool RFBoard_Dummy_InitBoard(void)
{
    return true;
}

bool RFBoard_Dummy_ConfigBoard(void)
{
    return true;
}

bool RFBoard_Dummy_SetPaBias(uint32_t bias)
{
    return true;
}

#ifdef USE_OSC_SParkle
const hardware_ident_t sparkle_desc =
{
    .creator = SParkleTRX_HW_CREATOR,
    .license = SParkleTRX_HW_LIC,
    .name = TRX_NAME_SParkle
};
#endif
const hardware_ident_t generic_desc =
{
    .creator = "K. Atanassov,M0NKA,www.m0nka.co.uk",
    .license = "CC BY-NC-SA 3.0",
    .name = TRX_NAME
};

const hardware_ident_t ddcduc_df8oe_desc =
{
    .creator = "DG8YGW, DF8OE et al",
    .license = "CERN-OHL v1.2",
    .name = TRX_NAME
};

/**
 *
 * Runs detection and first initialization of the connected RFBoard
 * This has to be called before using any other RFboard related functions
 * To have the board fully operational, RFBoard_Config_Board() has to be called, too.
 *
 */
bool RFBoard_Init_Board(void)
{

    // Initialize LO, by which we (at least for now) can detect the RF board
    Osc_Init();

    // we determine and set the correct RF board here
    switch(osc->type())
    {
        case OSC_SI5351A:           ts.rf_board = RF_BOARD_RS928; break;
        case OSC_DUCDDC_DF8OE:      ts.rf_board = RF_BOARD_DDCDUC_DF8OE; break;
        case OSC_DUCDDC_DDC2MODULE: ts.rf_board = RF_BOARD_DDCDUC_MCHF; break;
        case OSC_SI570:             ts.rf_board = RF_BOARD_MCHF; break;
        case OSC_SPARKLE:           ts.rf_board = RF_BOARD_SPARKLE; break;
        default:                    ts.rf_board = RF_BOARD_UNKNOWN;
    }


    switch(ts.rf_board)
    {
        case RF_BOARD_SPARKLE:
#ifdef USE_OSC_SParkle
            RFboard.pa_info=&SParkle_pa;
            RFboard.power_levelsInfo=&SParkle_power_levelsInfo;
            RFboard.EnableTx  = osc_SParkle_EnableTx;
            RFboard.EnableRx = osc_SParkle_EnableRx;
            RFboard.PrepareTx  = osc_SParkle_PrepareTx;
            RFboard.PrepareRx = osc_SParkle_PrepareRx;
            RFboard.ChangeFrequency = RFBoard_Dummy_ChangeFrequency;
            RFboard.InitBoard = RFBoard_Dummy_InitBoard;
            RFboard.ConfigBoard = SParkle_ConfigurationInit;
            RFboard.SetPABias = RFBoard_Dummy_SetPaBias;
            RFboard.SetPowerFactor = SParkle_SetTXpower;
            RFboard.description = &sparkle_desc;
            RFboard.iq_balance_required = false;
            RFboard.name = "SParkle RF";
            RFboard.is_ddcduc = true;
#endif

            break;

        case RF_BOARD_DDCDUC_DF8OE:
#ifdef USE_OSC_DUCDDC
            RFboard.pa_info=&Df8oe_DdcDuc_pa;
            RFboard.power_levelsInfo=&Df8oe_DdcDuc_power_levelsInfo;
            RFboard.EnableTx  = DucDdc_Df8oe_EnableTx;
            RFboard.EnableRx = DucDdc_Df8oe_EnableRx;
            RFboard.PrepareTx  = DucDdc_Df8oe_PrepareTx;
            RFboard.PrepareRx = DucDdc_Df8oe_PrepareRx;
            RFboard.ChangeFrequency = RFBoard_Dummy_ChangeFrequency;
            RFboard.InitBoard = RFBoard_Dummy_InitBoard;
            RFboard.ConfigBoard = DucDdc_Df8oe_Config;
            RFboard.SetPABias = RFBoard_Dummy_SetPaBias;
            RFboard.EnableCWCarrier = DucDdc_Df8oe_TxCWOnOff;
            RFboard.description = &ddcduc_df8oe_desc;
            RFboard.iq_balance_required = false;
            RFboard.name = "DDCDUC DF8OE RF";
            RFboard.is_ddcduc = true;
#endif
            break;
        case RF_BOARD_DDCDUC_MCHF:
        case RF_BOARD_MCHF:
        case RF_BOARD_RS928:
        default: // HACK: in case we don't detect a board, we still initialize to mcHF RF for now.
            RFboard.pa_info=&mchf_pa;
            RFboard.EnableTx  = Mchf_EnableTx;
            RFboard.EnableRx = Mchf_EnableRx;
            RFboard.PrepareTx  = Mchf_PrepareTx;
            RFboard.PrepareRx = Mchf_PrepareRx;
            RFboard.ChangeFrequency = Mchf_SetHWFiltersForFrequency;
            RFboard.InitBoard = Mchf_RfBoard_Init;
            RFboard.ConfigBoard = RFBoard_Dummy_ConfigBoard;
            RFboard.SetPABias = Mchf_RfBoard_SetPaBiasValue;
            RFboard.description = &generic_desc;

            // the settings below handle some variations between different hardware setups
            RFboard.iq_balance_required = ts.rf_board == RF_BOARD_DDCDUC_MCHF;
            RFboard.name = (ts.rf_board == RF_BOARD_RS928) ? "RS9xx RF" :
                    ( (ts.rf_board == RF_BOARD_DDCDUC_MCHF) ? "Mod. mcHF RF" : "mcHF RF");
            RFboard.is_ddcduc = ts.rf_board == RF_BOARD_DDCDUC_MCHF;
            RFboard.EnableCWCarrier = ts.rf_board == RF_BOARD_DDCDUC_MCHF? DucDdc_Df8oe_TxCWOnOff : NULL;
            RFboard.power_levelsInfo= ts.rf_board == RF_BOARD_DDCDUC_MCHF? &mchf_ddcduc_power_levelsInfo : &mchf_power_levelsInfo;
    }

    return RFboard.InitBoard();
}

/**
 * This will configure the rf board b4N_1472ased on the configuration data
 * to be called after RFBoard_Init_Board and after configuration data
 * has been restored from persistent storage
 *
 * @return true if everything went well
 */
bool RFBoard_Config_Board(void)
{
    bool retval = RFboard.ConfigBoard();

    osc->setPPM((float)ts.freq_cal/10.0);

    return retval;
}
