/*******************************************************************************************
 * 
 * IcbRemote:   Class to handle SCPI commands for the Instrument Control Board
 *              Created for Arduino using PlatformIO and C++17 support.
 * 
 * Author: Andrew Johnson
 * 
 * Attribution: Uses Jan Breuer's SCPI-PARSER framework
 *              https://github.com/j123b567/scpi-parser
 * 
 * March 2021
 * 
 *******************************************************************************************/
#include "IcbRemoteScpi.h"

/********************************************************************************************
 * void initialise()
 * 
 * Scope: public
 * Parameters: None
 * 
 * Returns: None
 * 
 * Description:
 * Initialises the SCPI framework
 * 
 ********************************************************************************************/
void IcbRemoteScpi::initialise() {
    SCPI_Init(&scpi_context,
        scpi_commands,
        &scpi_interface,
        scpi_units_def,
        SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
        scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
        scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);
}



scpi_result_t IcbRemoteScpi::icbSCPIControl([[maybe_unused]]scpi_t * context, [[maybe_unused]]scpi_ctrl_name_t ctrl, [[maybe_unused]]scpi_reg_val_t val) {
//    (void)context;
    return SCPI_RES_OK;
}

// Flush the output buffer?????
scpi_result_t icbSCPIFlush([[maybe_unused]]scpi_t * context) {

}

// Reset the instrument control board to default
scpi_result_t icbSCPIReset([[maybe_unused]]scpi_t * context) {
    
}

// Test features of board are ok
scpi_result_t icbSelfTest(scpi_t * context) {

}

// Handle errors
int icbSCPIError(scpi_t * context, int_fast16_t err) {

}
// Write response to output buffer
size_t icbSCPIWrite(scpi_t * context, const char * data, size_t len) {

}
