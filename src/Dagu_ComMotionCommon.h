#ifndef Dagu_ComMotionCommon_h
#define Dagu_ComMotionCommon_h

#include <Arduino.h>
// #include <Mainscreen_system.h>

/*****************************************
 *                                       *
 * COMMOTION COMMAND NUMBERS             *
 *                                       *
 *****************************************/

// These are the first byte of any command
// packet sent to the ComMotion via I2C
// or serial bus.

enum Dagu_ComMotionCmd
{
  DCC_UNKNOWN = 0,          // Relayed
  DCC_BASIC_CONFIG = 1,     // Relayed
  DCC_ENCODER_CONFIG = 2,   // Relayed
  DCC_MOTOR_CONTROL = 3,    // Relayed
  DCC_SERIAL_CONFIG = 4,    // Relayed
  DCC_SERIAL_SEND = 5,      // Not Relayed
  DCC_STATUS_REQUEST = 6,   // Not Relayed
  DCC_DEMO_ANGLE_UPDATE = 7,// Not Relayed
  DCC_EEROM_DEFAULTS = 10,  // Relayed,
  DCC_BEEP = 11,            // Not Relayed, later firmware only
  DCC_ECHO = 12,            // Not Relayed, later firmware only
  DCC_DEMO_SYNC = 15,       // Not Relayed,
  DCC_INTERNAL = 16,        // Not Relayed (OR'd with lower command),
  DCC_LIMIT = 32,           // commands this or above ignored,
  DCC_NONE = 255
};

/*****************************************
 *                                       *
 * COMMOTION MOTOR ERROR LOG FLAGS       *
 *                                       *
 *****************************************/

// These values indicate conditions such 
// as low battery voltage can be returned
// by the request status packet command

enum Dagu_ComMotionMotorError
{
    DCME_M1_OVER_CURRENT = B00000001,
    DCME_M2_OVER_CURRENT = B00000010,
    DCME_M3_OVER_CURRENT = B00000100,
    DCME_M4_OVER_CURRENT = B00001000,
    DCME_LOW_BATTERY = B00010000,
    DCME_LOW_BATTERY_SHUTDOWN = B00100000
};


/*****************************************
 *                                       *
 * COMMOTION BASIC MODES                 *
 *                                       *
 *****************************************/

// These are the only two basic modes:
// remotely controlled via Serial / I2c
// or demo.

enum Dagu_ComMotionBasicMode
{
  DCBCM_SERIAL_I2C = 0,
  DCBCM_DEMO = 1
};

/*****************************************
 *                                       *
 * COMMOTION WHEEL AND MOTOR CONFIG      *
 *                                       *
 *****************************************/

// The ComMotion can support a variety
// of configurations of wheels and motors:
enum Dagu_ComMotionConfig
{
  // In the case of Ommi and Mecanum modes,
  // the/ wheels are driven in coordination
  // to generate an omnidirectional force,
  // allowing the robot to move in any
  // direction. Thr host merely requests
  // the direction and speed and the
  // ComMotion does the Maths!
  DCC_3XOMNI = 0,
  DCC_4XOMNI = 1,
  DCC_MECANUM = 2,
  
  // For indepedent drive, the host decides
  // The individual motor speeds and
  // ComMotion attempts to mantain those
  // speeds under varying conditions:
  DCC_INDEPENDENT = 3,
  
  // ORing this flag with the other
  // flags disables the Odometer
  // encoder support, which in turn
  // disables the automatic speed control;
  // the speeds commanded by the host
  // go directly to the pwm outputs to the
  // motors.
  DCC_NO_ENCODERS = 16,
};

/*****************************************
 *                                       *
 * BEGENDIAN INTEGERS                    *
 *                                       *
 *****************************************/

// Most Arduino controllers are little endian
// i.e. in any multibyte number, the first byte
// has the smallest value (also called low byte
// first)
// ComMotion however, is bigendian, i.e.
// high byte first. This type allows seamless
// conversion between the two:

struct __attribute__((packed)) BigEndian16 
{
  uint8_t hi; uint8_t lo;

  BigEndian16(const uint8_t hi, const uint8_t lo) :
    hi(hi), lo(lo)
  {}

  BigEndian16(const uint16_t i) :
    hi(highByte(i)), lo(lowByte(i))
  {}

  // Get as little endian.
  uint16_t get()
  {
    return word(hi,lo);
  }

  void set(const uint16_t i)
  {
    hi = highByte(i);
    lo = lowByte(i);
  }
};

// Compile time size check.
static_assert(sizeof(BigEndian16) == 2, "BigEndian size borked!");

/*****************************************
 *                                       *
 * COMMAND PACKET BASE                   *
 *                                       *
 *****************************************/

// Every ComMotion command packet starts the
// same way, with the command number:

struct __attribute__((packed)) Dagu_ComMotionPacketBase 
{
  Dagu_ComMotionCmd cmd:8;
  Dagu_ComMotionPacketBase(Dagu_ComMotionCmd cmd) : cmd(cmd) {}
} ;


/*****************************************
 *                                       *
 * BASIC CONFIG PACKET                   *
 *                                       *
 *****************************************/

// The ComMotion mode, wheel and motor config,
// minimum battery, current warnings, and i2c addresses.

// The parameter payload and the command packet are declared
// separately, allowing the parameters to be stored in host
// RAM and saved to the NVR without the command number.
struct __attribute__((packed)) Dagu_ComMotionBasicConfig
{
  Dagu_ComMotionBasicMode mode:8;
  Dagu_ComMotionConfig config:8;
  uint8_t minBattery_dV;
  uint8_t maxCurrentM1_cA,maxCurrentM2_cA,maxCurrentM3_cA,maxCurrentM4_cA;
  uint8_t i2cOffset;
  uint8_t i2cMasterAddress;

  Dagu_ComMotionBasicConfig() {}
  
  Dagu_ComMotionBasicConfig
  (
    Dagu_ComMotionBasicMode mode,
    Dagu_ComMotionConfig config,
    uint8_t minBattery_dV,
    uint8_t maxCurrentM1_cA,
    uint8_t maxCurrentM2_cA,
    uint8_t maxCurrentM3_cA,
    uint8_t maxCurrentM4_cA,
    uint8_t i2cOffset,
    uint8_t i2cMasterAddress
  ) :
    mode(mode),
    config(config),
    minBattery_dV(minBattery_dV),
    maxCurrentM1_cA(maxCurrentM1_cA),
    maxCurrentM2_cA(maxCurrentM2_cA),
    maxCurrentM3_cA(maxCurrentM3_cA),
    maxCurrentM4_cA(maxCurrentM4_cA),
    i2cOffset(i2cOffset),
    i2cMasterAddress(i2cMasterAddress)
  {
    
  }
};

struct __attribute__((packed)) Dagu_ComMotionBasicConfigPacket  :
public Dagu_ComMotionPacketBase
{
  Dagu_ComMotionBasicConfigPacket
  (
    Dagu_ComMotionBasicMode mode,
    Dagu_ComMotionConfig config,
    uint8_t minBattery_dV,
    uint8_t maxCurrentM1_cA,
    uint8_t maxCurrentM2_cA,
    uint8_t maxCurrentM3_cA,
    uint8_t maxCurrentM4_cA,
    uint8_t i2cOffset,
    uint8_t i2cMasterAddress
  ) :
    Dagu_ComMotionPacketBase(DCC_BASIC_CONFIG),
    basicConfig(mode,config, minBattery_dV,maxCurrentM1_cA,maxCurrentM2_cA,maxCurrentM3_cA,maxCurrentM4_cA,i2cOffset,i2cMasterAddress)
  {}
  
  Dagu_ComMotionBasicConfigPacket
  (
    Dagu_ComMotionBasicConfig basicConfig
  ) :
    Dagu_ComMotionPacketBase(DCC_BASIC_CONFIG),
    basicConfig(basicConfig)
  {}

  Dagu_ComMotionBasicConfig basicConfig;
} ;

static_assert(sizeof(Dagu_ComMotionBasicConfigPacket) == 10, "Basic config packet size borked!");

/*****************************************
 *                                       *
 * ENCODER CONFIG PACKET                 *
 *                                       *
 *****************************************/

// Sets various parameters that allow the
// the ComMotion to correctly regulate the
// motor speed.

// The parameter payload and the command packet are declared
// separately, allowing the parameters to be stored in host
// RAM and saved to the NVR without the command number.

struct __attribute__((packed)) Dagu_ComMotionEncoderConfig
{
  BigEndian16 maxMotorRpm, encoderRes_x100;
  uint8_t reservePower_percent, maxStallTime_ms;
  Dagu_ComMotionEncoderConfig(uint16_t maxMotorRpm, uint16_t encoderRes_x100, uint8_t reservePower_percent, uint8_t maxStallTime_ms) :
    maxMotorRpm(maxMotorRpm), encoderRes_x100(encoderRes_x100), reservePower_percent(reservePower_percent), maxStallTime_ms(maxStallTime_ms)
  {}
  Dagu_ComMotionEncoderConfig() : maxMotorRpm(0), encoderRes_x100(0), reservePower_percent(0), maxStallTime_ms(0) {}
};

struct __attribute__((packed)) Dagu_ComMotionEncoderConfigPacket1  :
public Dagu_ComMotionPacketBase
{
  Dagu_ComMotionEncoderConfig encoderConfig;
  
  Dagu_ComMotionEncoderConfigPacket1
  (
    uint16_t maxMotorRpm, uint16_t encoderRes_x100, uint8_t reservePower_percent, uint8_t maxStallTime_ms
  ) :
    Dagu_ComMotionPacketBase(DCC_ENCODER_CONFIG),
    encoderConfig(maxMotorRpm, encoderRes_x100, reservePower_percent, maxStallTime_ms)
  {}
  
  Dagu_ComMotionEncoderConfigPacket1
  (
    const Dagu_ComMotionEncoderConfig& encoderConfig  ) :
    Dagu_ComMotionPacketBase(DCC_ENCODER_CONFIG),
    encoderConfig(encoderConfig)
  {}

} ;

static_assert(sizeof(Dagu_ComMotionEncoderConfigPacket1) == 7, "Encoder config packet size borked!");

/*****************************************
 *                                       *
 * ENCODER CONFIG 4 PACKET               *
 *                                       *
 *****************************************/

// As above for Encoder config, but this time
// allowing each of the four possible to be configured 
// indivdually; for builds including a mixture of motors
// and / or encoders.

struct __attribute__((packed)) Dagu_ComMotionEncoderConfigPacket4  :
public Dagu_ComMotionPacketBase
{
  Dagu_ComMotionEncoderConfig encoderConfigs[4];
  
  Dagu_ComMotionEncoderConfigPacket4
  (
    uint16_t maxMotorRpm0, uint16_t encoderRes0_x100, uint8_t reservePower0_percent, uint8_t maxStallTime0_secs,
    uint16_t maxMotorRpm1, uint16_t encoderRes1_x100, uint8_t reservePower1_percent, uint8_t maxStallTime1_secs,
    uint16_t maxMotorRpm2, uint16_t encoderRes2_x100, uint8_t reservePower2_percent, uint8_t maxStallTime2_secs,
    uint16_t maxMotorRpm3, uint16_t encoderRes3_x100, uint8_t reservePower3_percent, uint8_t maxStallTime3_secs
  ) :
    Dagu_ComMotionPacketBase(DCC_ENCODER_CONFIG),
    encoderConfigs({
      Dagu_ComMotionEncoderConfig(maxMotorRpm0, encoderRes0_x100, reservePower0_percent, maxStallTime0_secs), 
      Dagu_ComMotionEncoderConfig(maxMotorRpm1, encoderRes1_x100, reservePower1_percent, maxStallTime1_secs), 
      Dagu_ComMotionEncoderConfig(maxMotorRpm2, encoderRes2_x100, reservePower2_percent, maxStallTime2_secs), 
      Dagu_ComMotionEncoderConfig(maxMotorRpm3, encoderRes3_x100, reservePower3_percent, maxStallTime3_secs)
    })
  {}
  
  Dagu_ComMotionEncoderConfigPacket4
  (
    const Dagu_ComMotionEncoderConfig encoderConfigs[4]
  ) :
    Dagu_ComMotionPacketBase(DCC_ENCODER_CONFIG),
    encoderConfigs({encoderConfigs[0],encoderConfigs[1],encoderConfigs[2],encoderConfigs[3]})
  {}

} ;

static_assert(sizeof(Dagu_ComMotionEncoderConfigPacket4) == 25, "Encoder config quad packet size borked!");

/*****************************************
 *                                       *
 * ENCODER EXTENDED CONFIG PACKET        *
 *                                       *
 *****************************************/

// Sets additional parameters regarding how the ComMotion
// handles the encoders; only available on firmware 2.3.4+

// The parameter payload and the command packet are declared
// separately, allowing the parameters to be stored in host
// RAM and saved to the NVR without the command number.

enum Dagu_ComMotionEncoderFlags
{
  DEF_SIGNED = 0x0,
  DEF_ABSOLUTE = 0x1
};

struct __attribute__((packed)) Dagu_ComMotionEncoderConfigX
{
  Dagu_ComMotionEncoderFlags flags : 8;
  Dagu_ComMotionEncoderConfigX(Dagu_ComMotionEncoderFlags flags) :
    flags(flags)
  {}
  Dagu_ComMotionEncoderConfigX() : flags(0) {}
};

struct __attribute__((packed)) Dagu_ComMotionEncoderConfigPacketX1  :
public Dagu_ComMotionEncoderConfigPacket1
{
  Dagu_ComMotionEncoderConfigX encoderConfigX;
  
  Dagu_ComMotionEncoderConfigPacketX1
  (
    uint16_t maxMotorRpm, uint16_t encoderRes_x100, uint8_t reservePower_percent, uint8_t maxStallTime_ms, Dagu_ComMotionEncoderFlags flags
  ) :
    Dagu_ComMotionEncoderConfigPacket1 (
      maxMotorRpm, encoderRes_x100, reservePower_percent, maxStallTime_ms
    ),
    encoderConfigX(flags)
  {}
  
  Dagu_ComMotionEncoderConfigPacketX1
  (
    const Dagu_ComMotionEncoderConfig& encoderConfig,
    const Dagu_ComMotionEncoderConfigX& encoderConfigX
  ) :
    Dagu_ComMotionEncoderConfigPacket1(encoderConfig),
    encoderConfigX(encoderConfigX)
  {}
} ;

static_assert(sizeof(Dagu_ComMotionEncoderConfigPacketX1) == 8, "Encoder extended config packet size borked!");

/*****************************************
 *                                       *
 * ENCODER EXTENDED CONFIG 4 PACKET      *
 *                                       *
 *****************************************/

// As above for Encoder config, but this time
// allowing each of the four possible to be configured 
// indivdually; for builds including a mixture of motors
// and / or encoders.

struct __attribute__((packed)) Dagu_ComMotionEncoderConfigPacketX4  :
public Dagu_ComMotionEncoderConfigPacket4
{
  
  Dagu_ComMotionEncoderConfigX encoderConfigXs[4];
  
  Dagu_ComMotionEncoderConfigPacketX4
  (
    uint16_t maxMotorRpm0, uint16_t encoderRes0_x100, uint8_t reservePower0_percent, uint8_t maxStallTime0_secs, Dagu_ComMotionEncoderFlags flags0,
    uint16_t maxMotorRpm1, uint16_t encoderRes1_x100, uint8_t reservePower1_percent, uint8_t maxStallTime1_secs, Dagu_ComMotionEncoderFlags flags1,
    uint16_t maxMotorRpm2, uint16_t encoderRes2_x100, uint8_t reservePower2_percent, uint8_t maxStallTime2_secs, Dagu_ComMotionEncoderFlags flags2,
    uint16_t maxMotorRpm3, uint16_t encoderRes3_x100, uint8_t reservePower3_percent, uint8_t maxStallTime3_secs, Dagu_ComMotionEncoderFlags flags3
  ) :
    Dagu_ComMotionEncoderConfigPacket4(
      maxMotorRpm0, encoderRes0_x100, reservePower0_percent, maxStallTime0_secs,
      maxMotorRpm1, encoderRes1_x100, reservePower1_percent, maxStallTime1_secs,
      maxMotorRpm2, encoderRes2_x100, reservePower2_percent, maxStallTime2_secs,
      maxMotorRpm3, encoderRes3_x100, reservePower3_percent, maxStallTime3_secs
    ),
    encoderConfigXs({
      Dagu_ComMotionEncoderConfigX(flags0), 
      Dagu_ComMotionEncoderConfigX(flags1), 
      Dagu_ComMotionEncoderConfigX(flags2), 
      Dagu_ComMotionEncoderConfigX(flags3)
    })
  {}
  
  Dagu_ComMotionEncoderConfigPacketX4
  (
    Dagu_ComMotionEncoderConfig encoderConfigs[4],
    Dagu_ComMotionEncoderConfigX encoderConfigXs[4]
  ) :
    Dagu_ComMotionEncoderConfigPacket4(encoderConfigs),
    encoderConfigXs({encoderConfigXs[0],encoderConfigXs[1],encoderConfigXs[2],encoderConfigXs[3]})
  {}

} ;

static_assert(sizeof(Dagu_ComMotionEncoderConfigPacketX4) == 29, "Encoder config extended quad packet size borked!");

/*****************************************
 *                                       *
 * INDEPENDENT MOTOR CONTROL PACKET      *
 *                                       *
 *****************************************/

// Control the speed and direction of the motors individually,
// giving a separate speed for each of the four possible motors.
// -255..255 Positive => forward, Negative => reverse
// The actual PWM value supplied to the motors will be automatically
// adjusted according to the encoder config and encoder signals.

struct __attribute__((packed)) Dagu_ComMotionIndeMotorControlPacket  :
public Dagu_ComMotionPacketBase
{
  BigEndian16 motorSpeeds [4];
  Dagu_ComMotionIndeMotorControlPacket(const int m1Speed, const int m2Speed, const int m3Speed, const int m4Speed) :
    Dagu_ComMotionPacketBase(DCC_MOTOR_CONTROL),
    motorSpeeds({BigEndian16(m1Speed),BigEndian16(m2Speed),BigEndian16(m3Speed),BigEndian16(m4Speed)})
  {}
  Dagu_ComMotionIndeMotorControlPacket(const int motorSpeeds[4]) :
    Dagu_ComMotionPacketBase(DCC_MOTOR_CONTROL),
    motorSpeeds({BigEndian16(motorSpeeds[0]),BigEndian16(motorSpeeds[1]),BigEndian16(motorSpeeds[2]),BigEndian16(motorSpeeds[3])})
  {}
};

static_assert(sizeof(Dagu_ComMotionIndeMotorControlPacket) == 9, "Independent motor control packet size borked!");

/*****************************************
 *                                       *
 * OMNI / MECANUM MOTOR CONTROL PACKET   *
 *                                       *
 *****************************************/

// Control the velocity, direction of travel and angle of rotation of the robot.
// The ComMotion automatically translates the values into the different
// motor speeds need to achieve the desired trajectory.

struct __attribute__((packed)) Dagu_ComMotionOmniMotorControlPacket  :
public Dagu_ComMotionPacketBase
{
  Dagu_ComMotionOmniMotorControlPacket(const int velocity, const int angle, const int rotation) :
    Dagu_ComMotionPacketBase(DCC_MOTOR_CONTROL),
    velocity(velocity), angle(angle), rotation(rotation)
  {}
  BigEndian16 velocity, angle, rotation;
};

static_assert(sizeof(Dagu_ComMotionOmniMotorControlPacket) == 7, "Omni motor control packet size borked!");

/*****************************************
 *                                       *
 * SERIAL CONFIGURATION PACKET           *
 *                                       *
 *****************************************/

// Used to configure the serial comms parameters
// of the ComMotion - baud rate etc.

// The parameter payload and the command packet are declared
// separately, allowing the parameters to be stored in host
// RAM and saved to the NVR without the command number.

enum Dagu_ComMotionSerialMode
{
  DCSM_DATA_TO_MASTER = 0,
  DCSM_COMMANDS_ON_PORT1_DATA_TO_MASTER = 1,
  DCSM_COMMANDS_ON_PORT2_DATA_TO_MASTER = 2,
  DCSM_COMMANDS_ON_PORT1_DATA_TO_PORT1 = 3,
  DCSM_COMMANDS_ON_PORT2_DATA_TO_PORT2 = 4
};

struct __attribute__((packed)) Dagu_ComMotionSerialConfig
{
  BigEndian16 baudRatePort1;
  BigEndian16 baudRatePort2;
  Dagu_ComMotionSerialMode mode:8;

  Dagu_ComMotionSerialConfig
  (
    uint16_t baudRatePort1,
    uint16_t baudRatePort2,
    Dagu_ComMotionSerialMode mode
  ) :
    mode(mode),
    baudRatePort1(baudRatePort1),
    baudRatePort2(baudRatePort1)
  {
    
  }
};

struct __attribute__((packed)) Dagu_ComMotionSerialConfigPacket  :
public Dagu_ComMotionPacketBase
{
  Dagu_ComMotionSerialConfigPacket
  (
    uint16_t baudRatePort1,
    uint16_t baudRatePort2,
    Dagu_ComMotionSerialMode mode
  ) :
    Dagu_ComMotionPacketBase(DCC_SERIAL_CONFIG),
    config(baudRatePort1, baudRatePort2, mode)
  {}
  
  Dagu_ComMotionSerialConfigPacket
  (
    Dagu_ComMotionSerialConfig config
  ) :
    Dagu_ComMotionPacketBase(DCC_SERIAL_CONFIG),
    config(config)
  {}

  Dagu_ComMotionSerialConfig config;
} ;

static_assert(sizeof(Dagu_ComMotionSerialConfigPacket) == 6, "Serial config packet size borked!");

/*
  DCC_SERIAL_CONFIG = 4,    // Relayed
  DCC_SERIAL_SEND = 5,      // Not Relayed
*/

/*****************************************
 *                                       *
 * STATUS REQUEST PACKET                 *
 *                                       *
 *****************************************/

// Used to request information from the ComMotion, e.g. battery level,
// encoder values etc.

struct __attribute__((packed)) Dagu_ComMotionStatusRequestPacket  :
public Dagu_ComMotionPacketBase
{
  enum DataRequired
  {
    // Bit 0: Returns 8 bytes, the encoder count from each motor high byte first.
    // Bit 1: Resets all encoder counters. If bit 0 is high then the counters will be read before being reset.
    // Bit 2: Returns 8 bytes, the current draw of each motor high byte first.
    // Bit 3: If sent to MCU1: Returns 6 bytes, the analog inputs A3, A6 and A7 from MCU1, high byte first. A7 is battery voltage*.
    //        If sent to MCU2: Returns 1 byte MCU number **,
    // Bit 4: If sent to MCU1: Returns 1 byte MCU number **,
    //        If sent to MCU2: Returns 6 bytes, the analog inputs A3, A6 and A7 from MCU1, high byte first.
    // Bit 5: Returns 1 byte, the error log for the motors.
    // Bit 6: Clears the error logs. If bit 5 or 6 are high then those error logs will be read first.
    // Bit 7: Used to indicate internal communication - return the data to the other MCU, rather than the host MCU.
    DR_ENCODERS = 1, DR_ENCODERS_RESET = 2, DR_CURRENT_DRAW = 4,
    DR_ANALOG_VALS_MCU1 = 8, DR_MCU_NUM_MCU2 = 8, DR_ANALOG_VALS_MCU2 = 16, DR_MCU_NUM_MCU1 = 16,
    DR_MOTOR_ERR_LOG = 32, DR_CLEAR_MOTOR_ERR_LOG = 64, DR_INTERNAL = 128
  } dataRequired : 8;
  
  Dagu_ComMotionStatusRequestPacket(DataRequired dataRequired) :
    Dagu_ComMotionPacketBase(DCC_STATUS_REQUEST),
    dataRequired(dataRequired)
    {}

  static uint8_t expectedReturnSize(uint8_t mcuNum, DataRequired dataRequired)
  {
    uint8_t retVal = 0;
    if((dataRequired & DR_ENCODERS) == DR_ENCODERS)
    {
      retVal += 4;
    }
    if((dataRequired & DR_CURRENT_DRAW) == DR_CURRENT_DRAW)
    {
      retVal += 4;
    }
    if((mcuNum == 1 && (dataRequired & DR_ANALOG_VALS_MCU1) == DR_ANALOG_VALS_MCU1) || (mcuNum == 2 && (dataRequired & DR_ANALOG_VALS_MCU2) == DR_ANALOG_VALS_MCU2))
    {
      retVal += 6;
    }
    if((mcuNum == 2 && (dataRequired & DR_MCU_NUM_MCU2) == DR_MCU_NUM_MCU2) || (mcuNum == 1 && (dataRequired & DR_MCU_NUM_MCU1) == DR_MCU_NUM_MCU1))
    {
      retVal += 1;
    }
    if((dataRequired & DR_MOTOR_ERR_LOG) == DR_MOTOR_ERR_LOG)
    {
      retVal += 1;
    }
    return retVal;
  }

  uint8_t expectedReturnSize(uint8_t mcuNum)
  {
    return Dagu_ComMotionStatusRequestPacket::expectedReturnSize(mcuNum, dataRequired);
  }

};

static_assert(sizeof(Dagu_ComMotionStatusRequestPacket) == 2, "Status request packet size borked!");

struct __attribute__((packed)) Dagu_ComMotionStatusRequestPacketX :
public Dagu_ComMotionStatusRequestPacket
{
  enum DataRequiredX
  {
    DRX_MAX_PULSE = 1, DRX_PULSE = 2, DRX_PWM =4, DRX_STALLED = 8
  } dataRequiredX : 8;
  
  Dagu_ComMotionStatusRequestPacketX(DataRequired dataRequired, DataRequiredX dataRequiredX) :
    Dagu_ComMotionStatusRequestPacket(dataRequired),
    dataRequiredX(dataRequiredX)
    {}

  static uint8_t expectedReturnSize(uint8_t mcuNum, DataRequired dataRequired, DataRequiredX dataRequiredX)
  {
    uint8_t retVal = Dagu_ComMotionStatusRequestPacket::expectedReturnSize(mcuNum, dataRequired);
    if((dataRequiredX & DRX_MAX_PULSE) == DRX_MAX_PULSE)
    {
      retVal += 8;
    }
    if((dataRequiredX & DRX_PULSE) == DRX_PULSE)
    {
      retVal += 8;
    }
    if((dataRequiredX & DRX_PWM) == DRX_PWM)
    {
      retVal += 2;
    }
    if((dataRequiredX & DRX_STALLED) == DRX_STALLED)
    {
      retVal += 2;
    }
    return retVal;
  }

  uint8_t expectedReturnSize(uint8_t mcuNum)
  {
    return Dagu_ComMotionStatusRequestPacketX::expectedReturnSize(mcuNum, dataRequired, dataRequiredX);
  }

};

static_assert(sizeof(Dagu_ComMotionStatusRequestPacketX) == 3, "Extended status request packet size borked!");

/*
  DCC_EEROM_DEFAULTS = 10,  // Relayed,
*/


/*****************************************
 *                                       *
 * BEEP COMMAND PACKET                   *
 *                                       *
 *****************************************/

// Not supported prior to 2.3.4

// Causes just one MCU of the ComMotion to beep the requested
// number of times. Useful for testing communication from
// host to ComMotion MCUs.

struct __attribute__((packed)) Dagu_ComMotionBeepPacket  :
public Dagu_ComMotionPacketBase
{
  uint8_t numBeeps;
  Dagu_ComMotionBeepPacket(uint8_t numBeeps)  :
    Dagu_ComMotionPacketBase(DCC_BEEP),
    numBeeps(numBeeps)
  {
    
  }
} ;

static_assert(sizeof(Dagu_ComMotionBeepPacket ) == 2, "Beep command packet size borked!");

/*****************************************
 *                                       *
 * ECHO COMMAND PACKET                   *
 *                                       *
 *****************************************/

// Not supported prior to 2.3.4

// Sent to one of the ComMotion MCUs, causes the ComMotion to repeat
// on to the return address the data it has received in the packet.
// Useful for testing two way communication between host and ComMotion
// MCUs. The tail of the packet can include up to 30 bytes of data.

struct __attribute__((packed)) Dagu_ComMotionEchoPacket  :
public Dagu_ComMotionPacketBase
{
  uint8_t returnAddress;
  Dagu_ComMotionEchoPacket(uint8_t returnAddress)  :
    Dagu_ComMotionPacketBase(DCC_ECHO),
    returnAddress(returnAddress)
  {
  }
} ;

static_assert(sizeof(Dagu_ComMotionEchoPacket ) == 2, "Echo command packet size borked!");

#endif
