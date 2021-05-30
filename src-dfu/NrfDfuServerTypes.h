#pragma once

#include <functional>
#include <string>

#define NORDIC_SECURE_DFU_SERVICE "0000fe59-0000-1000-8000-00805f9b34fb"      // Service handle 0x000b
#define NORDIC_DFU_CONTROL_POINT_CHAR "8ec90001-f315-4f60-9fb8-838830daea50"  // Handle 0x000F
#define NORDIC_DFU_PACKET_CHAR "8ec90002-f315-4f60-9fb8-838830daea50"         // Handle 0x000D

#define FLASH_PAGE_SIZE 4096

// TODO: MTU Size will depend on platform (MacOs -.-), 20 byte chunk is a safe bet to start with
#define MTU_CHUNK 20

#define RESPONSE_LEN_CHECKSUM 8
#define RESPONSE_LEN_SELECT 12

namespace NativeDFU {

typedef std::function<void(std::string service, std::string characteristic, std::string data)> ble_write_t;

// * Opcodes, extended errors not implemented
typedef enum {
    PROT_VER_KEY = 0x00,
    CREATE_KEY = 0x01,
    PACKET_RECEIPT_NOTIF_REQ_KEY = 0x02,
    CALCULATE_CHECKSUM_KEY = 0x03,
    EXECUTE_KEY = 0x04,
    SELECT_OBJECT_KEY = 0x06,  // See object_type
    MTU_GET_KEY = 0x07,
    OBJECT_WRITE_KEY = 0x08,
    PING_KEY = 0x09,
    HW_VER_GET_KEY = 0x0A,
    FW_VER_GET_KEY = 0x0B,
    DFU_ABORT_KEY = 0x0C,
    RESPONSE_CODE_KEY = 0x60
} op_code_t;

typedef enum { COMMAND = 0x01, DATA = 0x02 } object_type_t;

// * Response codes, extended errors not implemented
typedef enum {
    INVALID_CODE_RESP = 0x00,
    SUCCESS_RESP = 0x01,
    OPCODE_NOT_SUP_RESP = 0x02,
    INVALID_PARAM_RESP = 0x03,
    INSUFF_RESOURCES_RESP = 0x04,
    INVALID_OBJ = 0x05,
    UNSUPP_TYPE_RESP = 0x07,
    OP_NOT_PERM_RESP = 0x08,
    OP_FAILED_RESP = 0x0A,
    EXT_ERROR_RESP = 0x0B
} response_code_t;

typedef union {
    struct {
        uint32_t maximum_size;
        uint32_t offset;
        uint32_t crc32;
    } select;

    struct {
        uint32_t offset;
        uint32_t crc32;
    } checksum;
} response_value_t;

typedef struct {
    uint8_t request_opcode;
    uint8_t result_code;
    response_value_t resp_val;
} control_point_response_t;

// * FSM States
typedef enum {
    DFU_IDLE,
    SET_NOTIF_VALUE,
    DATAFILE_CREATE_COM_OBJ,
    DATAFILE_WRITE_FILE,
    DATAFILE_REQ_CHECKSUM,
    DATAFILE_WRITE_EXECUTE,
    BINFILE_CREATE_DATA_OBJ,
    BINFILE_WRITE_MTU_CHUNK,
    BINFILE_REQ_CHECKSUM,
    BINFILE_WRITE_EXECUTE,
    BINFILE_WRITE_EXECUTE_FINAL,
    DFU_ERROR_CHECKSUM,
    DFU_ERROR,
    DFU_FINISHED
} state_t;

// * FSM Events
typedef enum {
    CHECKSUM_RECEIVED,
    SELECT_OBJ_RECEIVED,
    CREATE_SUC,
    PACKET_RECEIPT_NOTIF_REQ_SUC,
    EXECUTE_SUC,
    SELECT_OBJECT_SUC,  // See object_type
    RESPONSE_CODE_SUC,
    NO_EVENT,  // SOMETHING FAILED, process_response_data didn't set received_event
    ERROR_INV_LEN,
    ERROR_RECEIVED,
    ERROR_UNKNOW_REC_OP,     // SUCCESS received on unknown OP_CODE
    ERROR_NO_RESP_KEY,       // Received package doesn't start with RESPONSE_CODE_KEY
    ERROR_NOT_SUP_SERV_CHAR  // Not supported service or characteristic for notify
} event_t;

typedef enum { SUCCESS, RESP_ERR_INVALID } error_status_t;

}  // namespace NativeDFU
