/*
  Nice BusT4
  UART communication at 19200 8n1.
  A 519 us break (10 bits) is sent before each data packet.
  The packet fields that were decoded are described in packet_cmd_body_t.

  For Oview, 80 is always added to the address.
  The gate controller address stays unchanged.

Connection

BusT4                       ESP8266

Device side              Rx Tx GND
9  7  5  3  1
10 8  6  4  2
cable slot
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V

From the nice_dmbm_integration_protocol.pdf manual:

• ADR: the NICE network address where the target devices are located. It can range from 1 to 63 (1 to 3F).
This value must be in HEX. If the destination is the DIN-rail integration module, this value is 0 (adr = 0). If the destination
is an intelligent motor, this value is 1 (adr = 1).
• EPT: the address of the Nice motor within the ADR network. It can range from 1 to 127. This value must be in HEX.
• CMD: the command you want to send to the destination (ADR, EPT).
• PRF: profile setup command.
• FNC: the function you want to send to the destination (ADR, EPT).
• EVT: the event sent to the destination (ADR, EPT).
*/


#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"           // for adding Action
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h"              // parse strings using built-in helpers
#include <queue>                               // for queue support



namespace esphome {
namespace bus_t4 {

/* shorthand access to class members */
using namespace esphome::cover;
//using esp8266::timeoutTemplate::oneShotMs;


static const int _UART_NO=UART0; /* UART number */
static const int TX_P = 1;         /* Tx pin */
static const uint32_t BAUD_BREAK = 9200; /* baud rate used for the long pre-packet pulse */
static const uint32_t BAUD_WORK = 19200; /* operating baud rate */
static const uint8_t START_CODE = 0x55; /* packet start byte */

static const float CLOSED_POSITION_THRESHOLD = 0.007;  // Position threshold below which the gate is treated as fully closed
static const uint32_t POSITION_UPDATE_INTERVAL = 500;  // Drive position refresh interval, ms

/* ESP network settings
  Series can range from 0 to 63, default is 0.
  OVIEW addresses start from 8.

  When multiple drives with OXI are combined on the same network, different series values must be assigned.
  In that case, the OXI series must match the drive it controls.
*/






/* Packet message type.
  At the moment only CMD and INF matter here.
  The others were not deeply analyzed and their numeric values were not verified.
  6th byte of CMD and INF packets.
*/
enum mes_type : uint8_t {
  CMD = 0x01,  /* verified value, sends commands to the automation unit */
//  LSC = 0x02,  /* scenario list handling */
//  LST = 0x03,  /* automation list handling */
//  POS = 0x04,  /* request/change automation position */
//  GRP = 0x05,  /* send commands to an automation group with motor bitmask */
//  SCN = 0x06,  /* scenario handling */
//  GRC = 0x07,  /* send commands to groups created with Nice Screen Configuration Tool */
  INF = 0x08,  /* returns or sets device information */
//  LGR = 0x09,  /* group list handling */
//  CGR = 0x0A,  /* category groups created with Nice Screen Configuration Tool */
};




/*
command menu in the Oview hierarchy
9th byte of CMD packets
*/
enum cmd_mnu  : uint8_t {
  CONTROL = 0x01,
};


/* used in STA responses */
enum sub_run_cmd2 : uint8_t {
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
       OPENED = 0x04,
       CLOSED = 0x05,
      ENDTIME = 0x06,  // maneuver ended due to timeout
      STOPPED = 0x08,
  PART_OPENED = 0x10,  // partial opening
};

/* Errors */
enum errors_byte  : uint8_t {
  NOERR = 0x00, // no errors
  FD = 0xFD,    // command not supported by this device
  };

// Motor types
enum motor_type  : uint8_t {
  SLIDING = 0x01, 
  SECTIONAL = 0x02,
  SWING = 0x03,
  BARRIER = 0x04,
  UPANDOVER = 0x05, // up-and-over gate
  };

// 9th byte
enum whose_pkt  : uint8_t {
  FOR_ALL = 0x00,  /* packet for/from all devices */
  FOR_CU = 0x04,  /* packet for/from control unit */
  FOR_OXI = 0x0A,  /* packet for/from OXI receiver */
  };
	
// 10th byte of EVT GET/SET packets; only RUN was observed for CMD packets
enum command_pkt  : uint8_t {
  TYPE_M         = 0x00,   /* drive type request */
  INF_STATUS     = 0x01, // gate status (open/closed/stopped)
  WHO	         = 0x04,  /* who is on the bus? */
  MAC            = 0x07,    // mac address.
  MAN            = 0x08,   // manufacturer.
  PRD            = 0x09,   // product.
  INF_SUPPORT    = 0x10, // supported INF commands
  HWR            = 0x0a,   // hardware version.
  FRM            = 0x0b,   // firmware version.
  DSC            = 0x0c,   // description.
  CUR_POS        = 0x11,  // current logical automation position; DPRO924 expects position setup after this
  MAX_OPN        = 0x12,   // maximum possible encoder opening position
  POS_MAX        = 0x18,   // maximum encoder position (open)
  POS_MIN        = 0x19,   // minimum encoder position (closed)
  INF_P_OPN1     = 0x21, // partial opening 1
  INF_P_OPN2     = 0x22, // partial opening 2
  INF_P_OPN3     = 0x23, // partial opening 3
  INF_SLOW_OPN   = 0x24, // slowdown during opening
  INF_SLOW_CLS   = 0x25, // slowdown during closing
  OPN_OFFSET     = 0x28, /* opening delay */
  CLS_OFFSET     = 0x29, /* closing delay */
  OPN_DIS        = 0x2a, /* main settings - opening discharge */
  CLS_DIS        = 0x2b, /* main settings - closing discharge */
  REV_TIME       = 0x31, /* main settings - reverse time (brief inversion value) */
  OPN_PWR        = 0x4A,    /* main settings - force control - opening force */
  CLS_PWR        = 0x4B,    /* main settings - force control - closing force */
  SPEED_OPN      = 0x42,    /* main settings - speed setup - opening speed */
  SPEED_CLS      = 0x43,    /* main settings - speed setup - closing speed */
  SPEED_SLW_OPN  = 0x45,    /* main settings - speed setup - slow opening speed */
  SPEED_SLW_CLS  = 0x46,    /* main settings - speed setup - slow closing speed */
  OUT1           = 0x51,  /* output settings */
  OUT2           = 0x52,  /* output settings */
  LOCK_TIME      = 0x5A,  /* output settings - lock runtime */
  S_CUP_TIME     = 0x5C,  /* output settings - suction cup runtime */
  LAMP_TIME      = 0x5B,  /* output settings - courtesy light runtime */
  COMM_SBS       = 0x61,  /* command settings - step-by-step */
  COMM_POPN      = 0x62,  /* command settings - partial open */
  COMM_OPN       = 0x63,  /* command settings - open */
  COMM_CLS       = 0x64,  /* command settings - close */
  COMM_STP       = 0x65,  /* command settings - stop */
  COMM_PHOTO     = 0x68,  /* command settings - photo */
  COMM_PHOTO2    = 0x69,  /* command settings - photo2 */
  COMM_PHOTO3    = 0x6A,  /* command settings - photo3 */
  COMM_OPN_STP   = 0x6B,  /* command settings - stop while opening */
  COMM_CLS_STP   = 0x6C,  /* command settings - stop while closing */
  IN1            = 0x71,  /* input settings */
  IN2            = 0x72,  /* input settings */
  IN3            = 0x73,  /* input settings */
  IN4            = 0x74,  /* input settings */
  COMM_LET_OPN   = 0x78,  /* command settings - obstruction while opening */
  COMM_LET_CLS   = 0x79,  /* command settings - obstruction while closing */

  AUTOCLS        = 0x80,    /* main settings - auto close */
  P_TIME         = 0x81,    /* main settings - pause time */
  PH_CLS_ON      = 0x84,    /* main settings - close after photo - enabled */
  PH_CLS_VAR     = 0x86,    /* main settings - close after photo - mode */
  PH_CLS_TIME    = 0x85,    /* main settings - close after photo - wait time */
  ALW_CLS_ON     = 0x88,    /* main settings - always close - enabled */
  ALW_CLS_VAR    = 0x8A,    /* main settings - always close - mode */
  ALW_CLS_TIME   = 0x89,    /* main settings - always close - wait time */
  STAND_BY_ACT   = 0x8c,    /* main settings - standby mode - enabled ON/OFF */
  WAIT_TIME      = 0x8d,    /* main settings - standby mode - wait time */
  STAND_BY_MODE  = 0x8e,    /* main settings - standby mode - mode: safety=0x00, bluebus=0x01, all=0x02 */
  START_ON       = 0x90,    /* main settings - startup setting - enabled */
  START_TIME     = 0x91,    /* main settings - startup setting - startup time */
  SLOW_ON        = 0xA2,    /* main settings - slowdown */
  DIS_VAL        = 0xA4,    /* position - invalid disable value */

  BLINK_ON       = 0x94,    /* main settings - pre-flash - enabled */
  BLINK_OPN_TIME = 0x95,    /* main settings - pre-flash - opening time */
  BLINK_CLS_TIME = 0x99,    /* main settings - pre-flash - closing time */
  OP_BLOCK       = 0x9a,    /* main settings - operator block */
  KEY_LOCK       = 0x9c,    /* main settings - keypad lock */
  T_VAL          = 0xB1,    /* alarm threshold before service, in maneuver count */
  P_COUNT        = 0xB2,    /* dedicated partial counter */
  C_MAIN         = 0xB4,    /* cancel maintenance */
  DIAG_BB        = 0xD0,     /*   DIAGNOSTICS of bluebus devices */  
  INF_IO         = 0xD1,    /* input/output state */
  DIAG_PAR       = 0xD2,    /*  DIAGNOSTICS of other parameters   */
  
  
  
  


  

  CUR_MAN = 0x02,  // current maneuver
  SUBMNU  = 0x04,  // submenu
  STA = 0xC0,   // in-motion status
  MAIN_SET = 0x80,   // main settings
  RUN = 0x82,   // command to execute

  };	

	
/* run cmd, 11th byte of EVT packets */
enum run_cmd  : uint8_t {
  SET = 0xA9,  /* request parameter change */
  GET = 0x99,   /* request parameter read */
  GET_SUPP_CMD = 0x89, /* get supported commands */
  };


/* Command that must be executed.
11th byte of a CMD packet.
Used in requests and responses. */
enum control_cmd : uint8_t { 
  SBS = 0x01,    /* Step by Step */
  STOP = 0x02,   /* Stop */
  OPEN = 0x03,   /* Open */
  CLOSE = 0x04,  /* Close */
  P_OPN1 = 0x05, /* Partial opening 1 - wicket mode */
  P_OPN2 = 0x06, /* Partial opening 2 */
  P_OPN3 = 0x07, /* Partial opening 3 */
  RSP = 0x19, /* interface response confirming command reception */
  EVT = 0x29, /* interface response carrying requested information */
 
  P_OPN4 = 0x0b, /* Partial opening 4 - collective */
  P_OPN5 = 0x0c, /* Partial opening 5 - step-by-step priority */
  P_OPN6 = 0x0d, /* Partial opening 6 - open and lock */
  UNLK_OPN = 0x19, /* unlock and open */
  CLS_LOCK = 0x0E, /* close and lock */
  UNLCK_CLS = 0x1A, /* unlock and close */
  LOCK = 0x0F, /* lock */
  UNLOCK = 0x10, /* unlock */
  LIGHT_TIMER = 0x11, /* light timer */
  LIGHT_SW = 0x12, /* light on/off */
  HOST_SBS = 0x13, /* host SBS */
  HOST_OPN = 0x14, /* host open */
  HOST_CLS = 0x15, /* host close */
  SLAVE_SBS = 0x16, /* slave SBS */
  SLAVE_OPN = 0x17, /* slave open */
  SLAVE_CLS = 0x18, /* slave close */
  AUTO_ON = 0x1B, /* auto open active */
  AUTO_OFF = 0x1C, /* auto open inactive */
  
};
	
	
	
	
	
	
/* Reference information to better understand packet layout in the protocol */
// CMD request packet body
// packets with body size 0x0c = 12 bytes
	/*
struct packet_cmd_body_t {
  uint8_t byte_55;              // Header, always 0x55
  uint8_t pct_size1;                // body size (without header and CRC, total bytes minus three), 0x0c for commands
  uint8_t for_series;           // destination series, ff = broadcast
  uint8_t for_address;          // destination address, ff = broadcast
  uint8_t from_series;           // source series
  uint8_t from_address;          // source address
  uint8_t mes_type;           // message type, 1 = CMD, 8 = INF
  uint8_t mes_size;              // remaining byte count excluding the two trailing CRC bytes, 5 for commands
  uint8_t crc1;                // CRC1, XOR of the six previous bytes
  uint8_t cmd_mnu;                // command menu. cmd_mnu = 1 for control commands
  uint8_t setup_submnu;            // submenu, combined with command group defines message type
  uint8_t control_cmd;            // command to execute
  uint8_t offset;            // offset for responses, affects requests such as supported command list
  uint8_t crc2;            // crc2, XOR of the previous four bytes
  uint8_t pct_size2;            // body size (without header and CRC, total bytes minus three), 0x0c for commands

};





// RSP response packet body
// packets with body size 0x0e = 14 bytes
struct packet_rsp_body_t {
  uint8_t byte_55;              // Header, always 0x55
  uint8_t pct_size1;                // body size (without header and CRC, total bytes minus three), >= 0x0e
  uint8_t to_series;           // destination series, ff = broadcast
  uint8_t to_address;          // destination address, ff = broadcast
  uint8_t from_series;           // source series
  uint8_t from_address;          // source address
  uint8_t mes_type;           // message type, always 8 = INF for these packets
  uint8_t mes_size;              // remaining byte count excluding the two trailing CRC bytes, 5 for commands
  uint8_t crc1;                // CRC1, XOR of the six previous bytes
  uint8_t cmd_mnu;                // command menu. cmd_mnu = 1 for control commands
  uint8_t sub_inf_cmd;            // submenu the command came from, value is 0x80 lower than original submenu
  uint8_t sub_run_cmd;            // command received, value is 0x80 higher than the original command
  uint8_t hb_data;             // high byte of data
  uint8_t lb_data;            // low byte of data
  uint8_t err;               // errors
  uint8_t crc2;            // crc2, XOR of the previous four bytes
  uint8_t pct_size2;            // body size (without header and CRC, total bytes minus three), >= 0x0e

};
	
 // EVT response packet body with data
 
 struct packet_evt_body_t {
  uint8_t byte_55;              // Header, always 0x55
  uint8_t pct_size1;                // body size (without header and CRC, total bytes minus three), >= 0x0e
  uint8_t to_series;           // destination series, ff = broadcast
  uint8_t to_address;          // destination address, ff = broadcast
  uint8_t from_series;           // source series
  uint8_t from_address;          // source address
  uint8_t mes_type;           // message type, always 8 = INF for these packets
  uint8_t mes_size;              // remaining byte count excluding the two trailing CRC bytes, 5 for commands
  uint8_t crc1;                // CRC1, XOR of the six previous bytes
  uint8_t whose;                // packet owner: 00 = common, 04 = drive controller, 0A = OXI receiver
  uint8_t setup_submnu;            // submenu the command came from, equal to original submenu
  uint8_t sub_run_cmd;            // command being answered, value is 0x80 lower than the previously sent command
  uint8_t next_data;            // next data block
  uint8_t err;               // errors
  uint8_t data_blk;            // data block, may span multiple bytes
  uint8_t crc2;            // crc2, XOR of all previous bytes up to byte 9 (whose)
  uint8_t pct_size2;            // body size (without header and CRC, total bytes minus three), >= 0x0e

};
 
 
*/

enum position_hook_type : uint8_t {
     IGNORE = 0x00,
    STOP_UP = 0x01,
  STOP_DOWN = 0x02
 };

// Define the class, inheriting from Component and Cover
class NiceBusT4 : public Component, public Cover {
  public:
	
    // Drive settings
    bool autocls_flag; // auto close - L1
    bool photocls_flag; // close after photo - L2
    bool alwayscls_flag; // always close - L3
    bool init_ok = false; // drive identified at startup
    bool is_walky = false; // Walky uses a different position request command
    bool is_robus = false; // Robus does not require periodic position polling
    bool is_ro = false; // RO600 uses a different packet for position/motion status
		
    void setup() override;
    void loop() override;
    void dump_config() override; // log hardware information

    void send_raw_cmd(std::string data);
    void send_cmd(uint8_t data) {this->tx_buffer_.push(gen_control_cmd(data));}	
    void send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command,  std::string next_data, bool data_on, std::string data_command); // extended command
    void set_mcu(std::string command, std::string data_command); // command for the motor controller
    void set_status_update_interval(uint32_t status_update_interval) { this->status_update_interval_ = status_update_interval; }
		

    void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }
    
 /*   void set_update_interval(uint32_t update_interval) {  // drive status polling interval
      this->update_interval_ = update_interval;
    }*/

    cover::CoverTraits get_traits() override;

  protected:
    void control(const cover::CoverCall &call) override;
    void send_command_(const uint8_t *data, uint8_t len);
    void request_position(void);  // request current logical drive position
    void update_position(uint16_t newpos);  // update current logical drive position

    uint32_t last_position_time{0};  // time of the last current-position update
    uint32_t update_interval_{500};
    uint32_t last_update_{0};
    uint32_t last_uart_byte_{0};
    uint32_t last_received_status_millis{0};
    uint32_t status_update_interval_{500};

    CoverOperation last_published_op;  // last published state and position
    float last_published_pos{-1};

    void publish_state_if_changed(void);

    uint8_t position_hook_type{IGNORE};  // target-position hook flag and value
    uint16_t position_hook_value;

    uint8_t class_gate_ = 0x55; // 0x01 sliding, 0x02 sectional, 0x03 swing, 0x04 barrier, 0x05 up-and-over
//    uint8_t last_init_command_;
	
    bool init_cu_flag = false;	
    bool init_oxi_flag = false;	

	
    // UART-related variables
    uint8_t _uart_nr;
    uart_t* _uart = nullptr;
    uint16_t _max_opn = 0;  // maximum encoder or timer position
    uint16_t _pos_opn = 2048;  // open encoder/timer position, not available on all drives
    uint16_t _pos_cls = 0;  // closed encoder/timer position, not available on all drives
    uint16_t _pos_usl = 0;  // current logical encoder/timer position, not available on all drives
    // generated packet header settings
    uint8_t addr_from[2] = {0x00, 0x66}; // source address, BusT4 gateway
    uint8_t addr_to[2]; // destination address, controlled drive controller
    uint8_t addr_oxi[2]; // receiver address

    std::vector<uint8_t> raw_cmd_prepare (std::string data);             // prepare user-provided data for transmission

    // INF command generation
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len);	 // full form
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, 0x00, {0x00}, 0 );} // commands without data
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, std::vector<uint8_t> data){
	    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, next_data, data, data.size());} // commands with data
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data){
	    return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);} // addressed commands without data
    	    
    // CMD command generation
    std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);	    	
	
    void init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device );
    void send_array_cmd (std::vector<uint8_t> data);	
    void send_array_cmd (const uint8_t *data, size_t len);


    void parse_status_packet (const std::vector<uint8_t> &data); // parse a status packet
    
    void handle_char_(uint8_t c);                                         // received-byte handler
    void handle_datapoint_(const uint8_t *buffer, size_t len);          // received-data handler
    bool validate_message_();                                         // received-message validation

    std::vector<uint8_t> rx_message_;                          // received message accumulated byte by byte
    std::queue<std::vector<uint8_t>> tx_buffer_;             // outgoing command queue
    bool ready_to_tx_{true};	                           // whether transmission is currently allowed
	
    std::vector<uint8_t> manufacturer_ = {0x55, 0x55};  // manufacturer unknown during initialization
    std::vector<uint8_t> product_;
    std::vector<uint8_t> hardware_;
    std::vector<uint8_t> firmware_;
    std::vector<uint8_t> description_;	
    std::vector<uint8_t> oxi_product;
    std::vector<uint8_t> oxi_hardware;
    std::vector<uint8_t> oxi_firmware;
    std::vector<uint8_t> oxi_description;	

}; // class

} // namespace bus_t4
} // namespace esphome
