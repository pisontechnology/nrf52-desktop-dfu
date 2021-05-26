#include "NrfDfuServer.h"
#include "crc.h"
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

static std::string ToHex(const std::string &s, bool upper_case) {  // Used for debugging
    std::ostringstream ret;
    for (std::string::size_type i = 0; i < s.length(); ++i) {
        int z = s[i] & 0xff;
        ret << std::hex << std::setfill('0') << std::setw(2) << (upper_case ? std::uppercase : std::nouppercase) << z;
    }
    return ret.str();
}

using namespace NativeDFU;

NrfDfuServer::NrfDfuServer(ble_write_t write_command_p, ble_write_t write_request_p, const std::string &datafile_data_r,
                           const std::string &binfile_data_r)
    : state(DFU_IDLE),
      response{0},  //?Will this init. struct to 0?
      received_event(NO_EVENT),
      waiting_response(false),

      datafile_data(datafile_data_r),
      binfile_data(binfile_data_r),

      bin_bytes_written(0),
      bin_bytes_to_write(0),
      mtu_extra_bytes(0),
      mtu_chunks_remaing(0),
      mtu_last_chunk(false),

      crc32_result(0),
      write_command(write_command_p),
      write_request(write_request_p) {
    crcInit();  // Allows the usage of Fastcrc :D
}

NrfDfuServer::~NrfDfuServer() {}

// * Methods to send necessary data for DFU handshake

// ! Generates size_str with the uint16_t num_pcks as bytes, this ASSUMES LITTLE ENDIANNESS.
void NrfDfuServer::set_pck_notif_value(uint16_t num_pcks) {
    std::string size_str(reinterpret_cast<const char *>(&num_pcks), sizeof(num_pcks));
    this->write_procedure(std::string() + char(PACKET_RECEIPT_NOTIF_REQ_KEY) + size_str);
}

void NrfDfuServer::select_object(object_type_t obj_type) {
    this->write_procedure(std::string() + char(SELECT_OBJECT_KEY) + char(obj_type));
}

// ! Generates size_str with the uint32_t size as bytes, this ASSUMES LITTLE ENDIANNESS.
void NrfDfuServer::write_create_request(object_type_t obj_type, uint32_t size) {
    std::string opcode = std::string() + char(CREATE_KEY);
    switch (obj_type) {
        case COMMAND:
            opcode.append(std::string() + char(COMMAND));
            break;

        case DATA:
            opcode.append(std::string() + char(DATA));
            break;

        default:
            // std::cout << "I'm a donkey and wrongly called write_create_request. I should read DFU documentation" <<
            // std::endl;
            break;
    }
    std::string size_str(reinterpret_cast<const char *>(&size), sizeof(size));
    this->write_procedure(opcode + size_str);
}

void NrfDfuServer::write_packet(std::string data_send) {
    write_command(NORDIC_SECURE_DFU_SERVICE, NORDIC_DFU_PACKET_CHAR, data_send);
}

void NrfDfuServer::request_checksum() { this->write_procedure(std::string() + char(CALCULATE_CHECKSUM_KEY)); }

void NrfDfuServer::write_execute() { this->write_procedure(std::string() + char(EXECUTE_KEY)); }

void NrfDfuServer::write_procedure(std::string opcode_parameters) {
    // std::cout << "[WRITE_OPCODE] char-write-req: 0x000f  " << ToHex(opcode, true) << std::endl;
    this->write_request(NORDIC_SECURE_DFU_SERVICE, NORDIC_DFU_CONTROL_POINT_CHAR, opcode_parameters);
}

// * High level Public Methods to Handle FSM

void NrfDfuServer::run_dfu() {
    while (this->state != NativeDFU::DFU_FINISHED && this->state != NativeDFU::DFU_ERROR &&
           this->state != NativeDFU::DFU_ERROR_CHECKSUM) {
        this->run();
    }
}

// ! Will be called on a BLE reception via a thread, be careful with raceconditions and synchronization
void NrfDfuServer::notify(std::string service, std::string characteristic, std::string data) {
    if (service == NORDIC_SECURE_DFU_SERVICE && characteristic == NORDIC_DFU_CONTROL_POINT_CHAR) {
        if (data[0] == RESPONSE_CODE_KEY) {
            process_response_data(data);
            // std::cout << "Event Received  " << this->received_event << std::endl;
            std::lock_guard<std::mutex> guard(mutex_waiting_response);
            this->waiting_response = false;
            this->cv_waiting_response.notify_all();
            // std::cout << "Notified" << std::endl;
        } else {
            this->received_event = ERROR_NO_RESP_KEY;
            // std::cout << "Received Data not starting with response key" << std::endl;
        }
    } else {
        this->received_event = ERROR_NOT_SUP_SERV_CHAR;
        // std::cout << "Not Supported service or characteristic for notify " << std::endl;
    }
}

state_t NrfDfuServer::get_state() { return this->state; }

// * Methods to Handle FSM

void NrfDfuServer::run() {
    // std::cout << "Running FSM" << std::endl;
    this->manage_state();
    std::unique_lock<std::mutex> lock(mutex_waiting_response);
    cv_waiting_response.wait(lock, [&] { return !this->waiting_response; });
    this->event_handler();  // Notify Received
    // std::cout << "State update to " << this->state << std::endl;
}

void NrfDfuServer::manage_state() {
    static uint32_t i = 0;
    this->waiting_response = false;  // To avoid errors when maintaining and modifying code
    switch (this->state) {
        case DFU_IDLE:
            this->waiting_response = false;
            // do nothing
            break;

        case SET_NOTIF_VALUE:
            this->waiting_response = true;
            this->set_pck_notif_value(0);
            break;

        case DATAFILE_CREATE_COM_OBJ:
            this->waiting_response = true;
            this->write_create_request(NativeDFU::COMMAND, this->datafile_data.length());
            break;

        case DATAFILE_WRITE_FILE:
            this->waiting_response = false;  // Device does not respond until checksum request
            this->calculate_crc(this->datafile_data.c_str(), this->datafile_data.length());
            this->data_mtu_chunks_remaing = this->datafile_data.length() / MTU_CHUNK;
            this->data_mtu_extra_bytes = this->datafile_data.length() % MTU_CHUNK;
            for (i = 0; i < this->data_mtu_chunks_remaing; i++) {
                this->write_packet( std::string(&this->datafile_data.c_str()[this->datafile_data.length() + MTU_CHUNK * i], MTU_CHUNK));  // send data file
                //std::cout << " MTU chunk " << MTU_CHUNK << std::endl;
            }
            if (this->data_mtu_extra_bytes) {
                this->write_packet(std::string(&this->datafile_data.c_str()[this->datafile_data.length() + MTU_CHUNK * i], this->data_mtu_extra_bytes));  // send data file
                //std::cout << " Last MTU chunk " << this->data_mtu_extra_bytes << std::endl;
            }
            break;

        case DATAFILE_REQ_CHECKSUM:
        case BINFILE_REQ_CHECKSUM:
            this->waiting_response = true;
            this->request_checksum();
            break;

        case DATAFILE_WRITE_EXECUTE:
        case BINFILE_WRITE_EXECUTE:
            this->waiting_response = true;
            this->write_execute();
            break;

        case BINFILE_WRITE_EXECUTE_FINAL:
            this->waiting_response = false;  // * Final Execute does not respond!
            this->write_execute();
            break;

        case BINFILE_CREATE_DATA_OBJ:
            this->bin_bytes_to_write = FLASH_PAGE_SIZE;

            if ((this->binfile_data.length() - this->bin_bytes_written) <= FLASH_PAGE_SIZE) {
                this->bin_bytes_to_write = (this->binfile_data.length() - this->bin_bytes_written);
                this->mtu_last_chunk = true;
                // std::cout << " Last mtu chunk " << std::endl;
            }

            if (this->bin_bytes_to_write) {
                this->waiting_response = true;
                this->calculate_crc(
                    this->binfile_data.c_str(),
                    this->bin_bytes_written +
                        this->bin_bytes_to_write);  // CRC is for all the data written, not just the last flash page!
                this->write_create_request(NativeDFU::DATA, this->bin_bytes_to_write);
            }
            break;

        case BINFILE_WRITE_MTU_CHUNK:
            this->waiting_response = false;
            this->mtu_chunks_remaing = this->bin_bytes_to_write / MTU_CHUNK;
            this->mtu_extra_bytes = this->bin_bytes_to_write % MTU_CHUNK;
            for (i = 0; i < this->mtu_chunks_remaing; i++) {
                this->write_packet(
                    std::string(&this->binfile_data.c_str()[this->bin_bytes_written + MTU_CHUNK * i], MTU_CHUNK));
            }
            if (this->mtu_extra_bytes) {
                this->write_packet(std::string(&this->binfile_data.c_str()[this->bin_bytes_written + MTU_CHUNK * i],
                                               this->mtu_extra_bytes));
            }
            this->bin_bytes_written += this->bin_bytes_to_write;
            break;

        case DFU_FINISHED:
            break;
    }
}

void NrfDfuServer::event_handler() {
    switch (this->state) {
        case DFU_IDLE:
            this->state = SET_NOTIF_VALUE;
            // do nothing
            break;

        case SET_NOTIF_VALUE:
            if (this->received_event == PACKET_RECEIPT_NOTIF_REQ_SUC) {
                this->state = DATAFILE_CREATE_COM_OBJ;
            } else {
                this->state = DFU_ERROR;
                // std::cout << "Unknow event for the current state" << std::endl;
            }
            break;

        case DATAFILE_CREATE_COM_OBJ:
            if (this->received_event == CREATE_SUC) {
                this->state = DATAFILE_WRITE_FILE;
            } else {
                this->state = DFU_ERROR;
                // std::cout << "Unknow event for the current state" << std::endl;
            }
            break;

        case DATAFILE_WRITE_FILE:
            this->state = DATAFILE_REQ_CHECKSUM;
            break;

        case DATAFILE_REQ_CHECKSUM:
            if (this->received_event == CHECKSUM_RECEIVED) {
                if (this->checksum_match()) {
                    this->state = DATAFILE_WRITE_EXECUTE;
                } else {
                    this->state = DFU_ERROR_CHECKSUM;
                    // std::cout << "Invalid Checksum" << std::endl;
                }
            } else {
                this->state = DFU_ERROR;
                // std::cout << "Unknow event for the current state" << std::endl;
            }
            break;

        case DATAFILE_WRITE_EXECUTE:
            if (this->received_event == EXECUTE_SUC) {
                this->state = BINFILE_CREATE_DATA_OBJ;
            } else {
                this->state = DFU_ERROR;
                // std::cout << "Unknow event for the current state" << std::endl;
            }
            break;

        case BINFILE_CREATE_DATA_OBJ:
            if (this->received_event == CREATE_SUC) {
                this->state = BINFILE_WRITE_MTU_CHUNK;
            } else {
                this->state = DFU_ERROR;
                // std::cout << "Unknow event for the current state" << std::endl;
            }
            break;

        case BINFILE_WRITE_MTU_CHUNK:
            this->state = BINFILE_REQ_CHECKSUM;
            break;

        case BINFILE_REQ_CHECKSUM:
            if (this->received_event == CHECKSUM_RECEIVED) {
                if (this->checksum_match()) {
                    this->state = BINFILE_WRITE_EXECUTE;
                    // std::cout << "Received checksum: 0x" << std::hex << std::setfill('0') << std::setw(2)
                    //           << this->response.resp_val.checksum.crc32 << std::endl;
                } else {
                    this->state = DFU_ERROR_CHECKSUM;
                    // std::cout << "Invalid Checksum" << std::endl;
                }
            } else {
                this->state = DFU_ERROR;
                // std::cout << "Unknow event for the current state" << std::endl;
            }
            break;

        case BINFILE_WRITE_EXECUTE:
            if (this->received_event == EXECUTE_SUC) {
                this->state = (this->mtu_last_chunk) ? BINFILE_WRITE_EXECUTE_FINAL : BINFILE_CREATE_DATA_OBJ;
            } else {
                this->state = DFU_ERROR;
                // std::cout << "Unknow event for the current state" << std::endl;
            }
            break;
        case BINFILE_WRITE_EXECUTE_FINAL:
            if (this->received_event == EXECUTE_SUC) {
                this->state = DFU_FINISHED;
            } else {
                this->state = DFU_ERROR;
                // std::cout << "Unknow event for the current state" << std::endl;
            }
            break;

        case DFU_FINISHED:
            break;
    }
    this->waiting_response = false;
}

void NrfDfuServer::process_response_data(std::string data) {
    uint32_t response_value_len = 0;
    const uint32_t *response_data_p = nullptr;  // Will point to response value in the received data
    this->received_event = NO_EVENT;            // Should never be set!

    this->response.request_opcode = data[1];
    this->response.result_code = data[2];
    response_value_len = (data.length() - 3);

    if (this->response.result_code == SUCCESS_RESP) {
        if (this->response.request_opcode == CALCULATE_CHECKSUM_KEY) {  // Todo: Validate len
            response_data_p = reinterpret_cast<uint32_t *>(&data[3]);
            this->response.resp_val.checksum.offset = *response_data_p++;
            this->response.resp_val.checksum.crc32 = *response_data_p++;
            this->received_event = CHECKSUM_RECEIVED;
        } else if (this->response.request_opcode == SELECT_OBJECT_KEY) {  // Todo: Validate len
            response_data_p = reinterpret_cast<uint32_t *>(&data[3]);
            this->response.resp_val.select.maximum_size = *response_data_p++;
            this->response.resp_val.select.offset = *response_data_p++;
            this->response.resp_val.select.crc32 = *response_data_p++;
            this->received_event = SELECT_OBJ_RECEIVED;
        } else if (response_value_len) {
            // std::cout << " Response value length should be zero for other opcodes " << std::endl;
            this->received_event = ERROR_INV_LEN;
            // Do something
        } else {
            switch (response.request_opcode) {
                case CREATE_KEY:
                    this->received_event = CREATE_SUC;
                    break;

                case PACKET_RECEIPT_NOTIF_REQ_KEY:
                    this->received_event = PACKET_RECEIPT_NOTIF_REQ_SUC;
                    break;

                case EXECUTE_KEY:
                    this->received_event = EXECUTE_SUC;
                    break;

                case SELECT_OBJECT_KEY:
                    this->received_event = SELECT_OBJECT_SUC;
                    break;

                case RESPONSE_CODE_KEY:
                    this->received_event = RESPONSE_CODE_SUC;
                    break;

                default:
                    this->received_event = ERROR_UNKNOW_REC_OP;
                    break;
            }
        }

    } else {
        this->received_event = ERROR_RECEIVED;
        std::cout << " Non success code received " << std::endl;
    }
}

bool NrfDfuServer::checksum_match() {
    // std::cout << "CRC32 RESULT: 0x" << this->crc32_result << " RECEIVED CRC32: 0x"
    //           << this->response.resp_val.checksum.crc32 << std::endl;
    return this->crc32_result == this->response.resp_val.checksum.crc32;
}

void NrfDfuServer::calculate_crc(const char *data, size_t length) {
    // std::cout << "Calculating checksum of length: " << length << std::endl;
    // std::cout << ToHex( std::string(data,length), true) << std::endl;
    this->crc32_result = crcFast(reinterpret_cast<const unsigned char *>(data), length);
    // std::cout << "CRC for data to be sent is 0x" << std::hex << std::setfill('0') << std::setw(2) <<
    // this->crc32_result
    //           << std::endl;
}
