#ifndef ICB_REMOTE_SCPI_H
#define ICB_REMOTE_SCPI_H

#include <Arduino.h>
#define bool SCPI_bool
#include "scpi.h"
#undef bool
#include "IcbRemoteScpiCommands.h"

#define SCPI_INPUT_BUFFER_LENGTH 256
#define SCPI_ERROR_QUEUE_SIZE 10
#define SCPI_IDN1 "AJohnson"
#define SCPI_IDN2 "Instrument Control Board"
#define SCPI_IDN3 "0"
#define SCPI_IDN4 "01.0.0"

extern char scpi_input_buffer[];
extern scpi_error_t scpi_error_queue_data[];
extern scpi_t scpi_context;

class IcbRemoteScpi {
public:
    void initialise();

    scpi_result_t icbSelfTest(scpi_t * context);
    int icbSCPIError(scpi_t * context, int_fast16_t err);
    size_t icbSCPIWrite(scpi_t * context, const char * data, size_t len);
    scpi_result_t icbSCPIControl(scpi_t * context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val);
    scpi_result_t icbSCPIFlush(scpi_t * context);
    scpi_result_t icbSCPIReset(scpi_t * context);

private:

};

#endif // ICB_REMOTE_SCPI_H