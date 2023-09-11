#ifndef _XTM_SERIAL_MODBUS_H
#define _XTM_SERIAL_MODBUS_H

/* RTU SUPPORTS - User specifics
 * Note: MODBUS_RTU_FLAG must not same as any
 * UIF_* uart_info flags defined in "serial_core.h"
 * ******************************************************************** */
#define MODBUS_RTU_FLAG (1<<15) /* 1. Indicate uart does support rtu    */
                                /*    stored in uart_port.info->flags   */
#define MODBUS_RTU_GET  0x4D47  /* 2. Check rtu support state of uart   */
#define MODBUS_RTU_SET  0x4D52  /* 3. Enable/Disable rtu support of Uart*/
#define MODBUS_TXENB    0x4D53  /* 4. Cmd 'modbus set'. Stored value in */
/*                                    UART_SCR bit 0                    */
/* -------------------------------------------------------------------- */

#endif  /* _XTM_SERIAL_MODBUS_H */
