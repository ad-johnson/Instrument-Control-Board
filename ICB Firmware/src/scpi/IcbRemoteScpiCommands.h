#ifndef ICB_REMOTE_SCPI_COMMANDS_H
#define ICB_REMOTE_SCPI_COMMANDS_H

#include "scpi.h"

scpi_result_t icbSelfTest(scpi_t * context);
int icbSCPIError(scpi_t * context, int_fast16_t err);
size_t icbSCPIWrite(scpi_t * context, const char * data, size_t len);
scpi_result_t icbSCPIControl(scpi_t * context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val);
scpi_result_t icbSCPIFlush(scpi_t * context);
scpi_result_t icbSCPIReset(scpi_t * context);


scpi_interface_t scpi_interface {
    icbSCPIError,
    icbSCPIWrite,
    icbSCPIControl,
    icbSCPIFlush,
    icbSCPIReset    // Handles *RST
};

const scpi_command_t scpi_commands[] {
    /* IEEE Mandated Commands (SCPI std V1999.0 4.1.1) */
    {"*CLS", SCPI_CoreCls, 0},      // Handled by framework
    {"*ESE", SCPI_CoreEse, 0},      // Handled by framework
    {"*ESE?", SCPI_CoreEseQ, 0},    // Handled by framework; calls icbSCPIWrite to return ESE
    {"*ESR?", SCPI_CoreEsrQ, 0},    // Handled by framework; calls icbSCPIWrite to return ESR
    {"*IDN?", SCPI_CoreIdnQ, 0},    // Handled by framework; calls icbSCPIWrite to return ESE
    {"*OPC", SCPI_CoreOpc, 0},      // Handled by framework
    {"*OPC?", SCPI_CoreOpcQ, 0},    // Handled by framework; calls icbSCPIWrite to return ESE
    {"*RST", SCPI_CoreRst, 0},      // calls interface->icbSCPIReset
    {"*SRE", SCPI_CoreSre, 0},      // Handled by framework
    {"*SRE?", SCPI_CoreSreQ, 0},    // Handled by framework; calls icbSCPIWrite to return ESE
    {"*STB?", SCPI_CoreStbQ, 0},    // Handled by framework; calls icbSCPIWrite to return ESE
    {"*TST?", icbSelfTest, 0},      // Self-Test
    {"*WAI", SCPI_CoreWai, 0},      // Handled by framework, assumes instrument does not have overlapping commands

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */
    {"SYSTem:ERRor[:NEXT]?", SCPI_SystemErrorNextQ, 0},     // Handled by framework; Investigate for device dependent error info.
    {"SYSTem:ERRor:COUNt?", SCPI_SystemErrorCountQ, 0},     // Handled by framework
    {"SYSTem:VERSion?", SCPI_SystemVersionQ, 0},            // Handled by framework

    // These are deliberately not implemented.
    /*
    {"STATus:OPERation?", scpi_stub_callback, 0},
    {"STATus:OPERation:EVENt?", scpi_stub_callback, 0},
    {"STATus:OPERation:CONDition?", scpi_stub_callback, 0},
    {"STATus:OPERation:ENABle", scpi_stub_callback, 0},
    {"STATus:OPERation:ENABle?", scpi_stub_callback, 0},
    */
    {"STATus:QUEStionable[:EVENt]?", SCPI_StatusQuestionableEventQ, 0}, // Handled by framework, returns the event register
    // {"STATus:QUEStionable:CONDition?", scpi_stub_callback, 0},       // Deliberately not implemented
    {"STATus:QUEStionable:ENABle", SCPI_StatusQuestionableEnable, 0},   // Handled by framework
    {"STATus:QUEStionable:ENABle?", SCPI_StatusQuestionableEnableQ, 0}, // Handled by framework

    {"STATus:PRESet", SCPI_StatusPreset, 0},                            // Handled by framework

    /* Instrument Control Board commands */

    SCPI_CMD_LIST_END
};

#endif // ICB_REMOTE_SCPI_COMMANDS_H