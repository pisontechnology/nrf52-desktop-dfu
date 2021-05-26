#pragma once

#include "NrfDfuServerTypes.h"
#include <condition_variable>
#include <mutex>
#include <string>

namespace NativeDFU {
class NrfDfuServer {
  public:
    /**
     * NrfDfuServer::NrfDfuServer()
     *
     * Constructor, will initialize crc library & variables
     *
     * @param write_command_p: callback to be called for writing a ble command
     * @param write_request_p: callback to be called for writing a ble request
     * @param datafile_data_r: [in] String containing the datafile DATA used for DFU. // !THIS IS NOT THE PATH
     * @param binfile_data_r: [in] String containing the binfile DATA used for DFU. // !THIS IS NOT THE PATH
     */
    NrfDfuServer(ble_write_t write_command_p, ble_write_t write_request_p, const std::string &datafile_data_r,
                 const std::string &binfile_data_r);

    /**
     * NrfDfuServer::~NrfDfuServer()
     *
     * Destructor
     *
     */
    ~NrfDfuServer();

    /**
     * NrfDfuServer::run_dfu
     *
     * Public method, will carry out the whole DFU process. Abstracting the user from internal functionality
     *
     */
    void run_dfu();

    /**
     * NrfDfuServer::notify
     *
     * This function notifies the FSM of a BLE package reception. The raw data is processed and saved in
     * control_point_response_t response;
     *
     * @param service: BLE service & characteristic which sent data
     * @param characteristic: BLE service & characteristic which sent data
     * @param data: Raw data received via BLE
     */
    void notify(std::string service, std::string characteristic, std::string data);

    /**
     * NrfDfuServer::get_state
     *
     * Getter returns the current state of the FSM.
     *
     * @return state_t: The current of the FSM
     */
    state_t get_state();

  private:
    // * Methods to send necessary data for DFU handshake

    /**
     * NrfDfuServer::set_pck_notif_value
     *
     * Sets number of packages to receive before generating a notification. THe DFU default value is 0, just incase call
     * this function before carrying out any DFU operation
     *
     * @param num_pcks: Number of packages to receive before generating a notification
     */
    void set_pck_notif_value(uint16_t num_pcks);

    /**
     * NrfDfuServer::select_object
     *
     * Selects the last object with the given type that was sent. This can be used to continue with a DFU on power down
     * or interruption mid transmission (Not supported in this DFU Implementation)
     *
     * @param obj_type: Defines the type of object to be create, can be command or data object
     */
    void select_object(object_type_t obj_type);

    /**
     * NrfDfuServer::write_create_request
     *
     * Carries out a Create Procedure: Create an object with the given type and selects it. Removes an old object of the
     * same type (if such an object exists).
     *
     * @param obj_type: Defines the type of object to be create, can be command or data object
     * @param size: Object size in little endian
     */
    void write_create_request(object_type_t obj_type, uint32_t size);

    /**
     * NrfDfuServer::write_packet
     *
     * Writes to the DFU Packet Characteristic. This characteristic receives data for Device Firmware Updates as DFU
     * packets.
     *
     * @param data_send: String containing Bytes to send
     */
    void write_packet(std::string data_send);

    /**
     * NrfDfuServer::request_checksum
     *
     * Requests the checksum of the current object. The checksum is reset after sending an Execute command.
     * IMPORTANT: Checksum will be carried on the WHOLE object. For example on the bin file the checksum will be carried
     * on all the FLASH_PAGES received NOT ONLY THE LAST ONE.
     *
     */
    void request_checksum();

    /**
     * NrfDfuServer::write_execute
     *
     * Executes the last object that was sent. This must be called after sending data and validating received checksum.
     * Must also be called after last bin file data
     *
     */
    void write_execute();

    /**
     * NrfDfuServer::write_procedure
     *
     * Writes to the DFU Control Point Characteristic. This characteristic is used to control the state of
     * the DFU process. All DFU procedures are requested by writing to this characteristic.
     *
     * @param opcode: String containing [Control Point OPCODE] + [Control Point Parameters] (optional)
     */
    void write_procedure(std::string opcode_parameters);

    // * Methods to Handle FSM

    /**
     * NrfDfuServer::run
     *
     * Internal function which runs one step of the FSM. Calls manage_state and event_handler, waiting for a response if
     * necessary.
     *
     */
    void run();

    /**
     * NrfDfuServer::manage_state
     *
     * This function runs the FSM for each state, carrying out the corresponding action. The bool waiting_response will
     * used by the class to know that is must wait for a notification before continuing running.
     *
     * IMPORTANT: The notify method will be called asynchronous on a thread. The notify method WILL MODIFY
     * this->waiting_response.
     * IMPORTANT: In the manage state method this->waiting_response should be written BEFORE SENDING
     * ANYTHING OVER BLE. If not notify can set this->waiting_response before manage_state and lock the FSM.
     *
     * Example of how the lock could happen. Imagine the following two lines of code in this method
     *  -this->write_create_request()
     *  -this->waiting_response = true; //Should wait for a response
     *
     * What COULD happen which would lock the FSM.
     *  -Main thread: this->write_create_request() called
     *  -Notification thread: received asynchronously which sets this->waiting received = false;
     *  -Main thread: this->waiting_response = true; //Notification already received no one to set it to false. FSM
     * LOCKED
     */
    void manage_state();

    /**
     * NrfDfuServer::event_handler
     *
     * This function will handle received events and carryout state transitions. For some states no event is necessary
     * for transition (this->waiting_response = false), the event handler should still be called as it will change to
     * the next state.
     *
     */
    void event_handler();

    /**
     * NrfDfuServer::process_response_data
     *
     * This function will process the data received via BLE coming from the Device and load it into
     * control_point_response_t response. This will later be used to generate the corresponding events.
     *
     * @param data: Raw data received via BLE.
     */
    void process_response_data(std::string data);

    // * Methods for checksum validation

    /**
     * NrfDfuServer::checksum_match
     *
     * This function will compare the stored checksum(crc32_result) with the received checksum via BLE
     *      -uint32_t this->crc32_result: Calculated by calling NrfDfuServer::calculate_cr()
     *      -uint32_t this->response.resp_val.checksum.crc32: Received checksum
     *
     * @param data: Raw data received via BLE.
     */
    bool checksum_match();

    /**
     * NrfDfuServer::calculate_crc
     *
     * Calculates the crc of the data and saves it to this->crc32_result.
     *
     * @param data: Data for which the CRC will be calculated
     * @param length: Length of the data for which the CRC will be calculated
     */
    void calculate_crc(const char *data, size_t length);

    // * FSM Management Variables
    state_t state;
    control_point_response_t response;
    event_t received_event;

    // * Synchronization with BLE thread for notification variables
    bool waiting_response;
    std::mutex mutex_waiting_response;
    std::condition_variable cv_waiting_response;

    // * Files data in std::string format: Reference used to avoid copy constructor
    const std::string &datafile_data;
    const std::string &binfile_data;

    // * Data file sending variables
    uint32_t data_mtu_extra_bytes;
    uint32_t data_mtu_chunks_remaing;

    // * Bin file sending variables
    uint32_t bin_bytes_written;   // Total bin_bytes_written
    uint32_t bin_bytes_to_write;  // Bytes to write on mtu cycle
    uint32_t mtu_extra_bytes;
    uint32_t mtu_chunks_remaing;
    bool mtu_last_chunk;

    // * CRC Result is calculated and stored here before sending data
    uint32_t crc32_result;

    // * Callbacks to write commands & request: This allows the DFU Server to be agnostic from the BLE implementation
    ble_write_t write_command;
    ble_write_t write_request;
};

}  // namespace NativeDFU
