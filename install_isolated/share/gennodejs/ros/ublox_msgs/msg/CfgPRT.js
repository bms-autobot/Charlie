// Auto-generated. Do not edit!

// (in-package ublox_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CfgPRT {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.portID = null;
      this.reserved0 = null;
      this.txReady = null;
      this.mode = null;
      this.baudRate = null;
      this.inProtoMask = null;
      this.outProtoMask = null;
      this.reserved4 = null;
      this.reserved5 = null;
    }
    else {
      if (initObj.hasOwnProperty('portID')) {
        this.portID = initObj.portID
      }
      else {
        this.portID = 0;
      }
      if (initObj.hasOwnProperty('reserved0')) {
        this.reserved0 = initObj.reserved0
      }
      else {
        this.reserved0 = 0;
      }
      if (initObj.hasOwnProperty('txReady')) {
        this.txReady = initObj.txReady
      }
      else {
        this.txReady = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('baudRate')) {
        this.baudRate = initObj.baudRate
      }
      else {
        this.baudRate = 0;
      }
      if (initObj.hasOwnProperty('inProtoMask')) {
        this.inProtoMask = initObj.inProtoMask
      }
      else {
        this.inProtoMask = 0;
      }
      if (initObj.hasOwnProperty('outProtoMask')) {
        this.outProtoMask = initObj.outProtoMask
      }
      else {
        this.outProtoMask = 0;
      }
      if (initObj.hasOwnProperty('reserved4')) {
        this.reserved4 = initObj.reserved4
      }
      else {
        this.reserved4 = 0;
      }
      if (initObj.hasOwnProperty('reserved5')) {
        this.reserved5 = initObj.reserved5
      }
      else {
        this.reserved5 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CfgPRT
    // Serialize message field [portID]
    bufferOffset = _serializer.uint8(obj.portID, buffer, bufferOffset);
    // Serialize message field [reserved0]
    bufferOffset = _serializer.uint8(obj.reserved0, buffer, bufferOffset);
    // Serialize message field [txReady]
    bufferOffset = _serializer.uint16(obj.txReady, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint32(obj.mode, buffer, bufferOffset);
    // Serialize message field [baudRate]
    bufferOffset = _serializer.uint32(obj.baudRate, buffer, bufferOffset);
    // Serialize message field [inProtoMask]
    bufferOffset = _serializer.uint16(obj.inProtoMask, buffer, bufferOffset);
    // Serialize message field [outProtoMask]
    bufferOffset = _serializer.uint16(obj.outProtoMask, buffer, bufferOffset);
    // Serialize message field [reserved4]
    bufferOffset = _serializer.uint16(obj.reserved4, buffer, bufferOffset);
    // Serialize message field [reserved5]
    bufferOffset = _serializer.uint16(obj.reserved5, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CfgPRT
    let len;
    let data = new CfgPRT(null);
    // Deserialize message field [portID]
    data.portID = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved0]
    data.reserved0 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [txReady]
    data.txReady = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [baudRate]
    data.baudRate = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [inProtoMask]
    data.inProtoMask = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [outProtoMask]
    data.outProtoMask = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [reserved4]
    data.reserved4 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [reserved5]
    data.reserved5 = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ublox_msgs/CfgPRT';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3679d3ad278d5329fb8543226a55acca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # CFG-PRT (0x06 0x00)
    # Port Configuration for UART
    #
    # Several configurations can be concatenated to one input message. In this case the payload
    # length can be a multiple of the normal length (see the other versions of CFG-PRT). Output
    # messages from the module contain only one configuration unit.
    #
    
    uint8 CLASS_ID = 6
    uint8 MESSAGE_ID = 0
    
    uint8 portID              # Port Identifier Number (= 1 or 2 for UART ports)
    uint8 PORT_ID_UART1 = 1
    uint8 PORT_ID_UART2 = 2
    uint8 PORT_ID_USB = 3
    
    uint8 reserved0           # Reserved
    
    uint16 txReady            # reserved (Alwyas set to zero) up to Firmware 7.01,
                              # TX ready PIN configuration (since Firmware 7.01)
    uint16 TX_READY_EN = 1                    # Enable TX ready feature for this port
    uint16 TX_READY_POLARITY_HIGH_ACTIVE = 0  # Polarity High-active
    uint16 TX_READY_POLARITY_LOW_ACTIVE = 2   # Polarity Low-active
    uint16 TX_READY_PIN_SHIFT = 2             # PIO to be used (must not be in use already by another function)
    uint16 TX_READY_PIN_MASK = 124            #
    uint16 TX_READY_THRES_SHIFT = 7           # Threshold
    uint16 TX_READY_THRES_MASK = 65408        # The given threshold is multiplied by 8 bytes.
                                              # The TX ready PIN goes active after >= thres*8 bytes are pending for the port and going inactive after the last
                                              # pending bytes have been written to hardware (0-4 bytes before end of stream).
    
    uint32 mode               # A bit mask describing the UART mode
    uint32 MODE_RESERVED1 = 16                # Default 1 for compatibility with A4
    uint32 MODE_CHAR_LEN_MASK = 192           # Character Length
    uint32 MODE_CHAR_LEN_5BIT = 0               # 5bit (not supported)
    uint32 MODE_CHAR_LEN_6BIT = 64              # 6bit (not supported)
    uint32 MODE_CHAR_LEN_7BIT = 128             # 7bit (supported only with parity)
    uint32 MODE_CHAR_LEN_8BIT = 192             # 8bit
    uint32 MODE_PARITY_MASK = 3584            #
    uint32 MODE_PARITY_EVEN = 0                 # Even Parity
    uint32 MODE_PARITY_ODD = 512                # Odd Parity
    uint32 MODE_PARITY_NO = 2048                # No Parity
    uint32 MODE_STOP_BITS_MASK = 12288        # Number of Stop Bits
    uint32 MODE_STOP_BITS_1 = 0                 # 1 Stop Bit
    uint32 MODE_STOP_BITS_15 = 4096             # 1.5 Stop Bit
    uint32 MODE_STOP_BITS_2 = 8192              # 2 Stop Bit
    uint32 MODE_STOP_BITS_05 = 12288            # 0.5 Stop Bit
    
    uint32 baudRate           # Baudrate in bits/second [bits/s]
    
    uint16 inProtoMask        # A mask describing which input protocols are active.
                              # Each bit of this mask is used for a protocol.
                              # Through that, multiple protocols can be defined
                              # on a single port.
    
    uint16 outProtoMask       # A mask describing which output protocols are active.
                              # Each bit of this mask is used for a protocol.
                              # Through that, multiple protocols can be defined
                              # on a single port.
    
    uint16 PROTO_UBX = 1
    uint16 PROTO_NMEA = 2
    uint16 PROTO_RTCM = 4
    
    uint16 reserved4          # Always set to zero
    uint16 reserved5          # Always set to zero
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CfgPRT(null);
    if (msg.portID !== undefined) {
      resolved.portID = msg.portID;
    }
    else {
      resolved.portID = 0
    }

    if (msg.reserved0 !== undefined) {
      resolved.reserved0 = msg.reserved0;
    }
    else {
      resolved.reserved0 = 0
    }

    if (msg.txReady !== undefined) {
      resolved.txReady = msg.txReady;
    }
    else {
      resolved.txReady = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.baudRate !== undefined) {
      resolved.baudRate = msg.baudRate;
    }
    else {
      resolved.baudRate = 0
    }

    if (msg.inProtoMask !== undefined) {
      resolved.inProtoMask = msg.inProtoMask;
    }
    else {
      resolved.inProtoMask = 0
    }

    if (msg.outProtoMask !== undefined) {
      resolved.outProtoMask = msg.outProtoMask;
    }
    else {
      resolved.outProtoMask = 0
    }

    if (msg.reserved4 !== undefined) {
      resolved.reserved4 = msg.reserved4;
    }
    else {
      resolved.reserved4 = 0
    }

    if (msg.reserved5 !== undefined) {
      resolved.reserved5 = msg.reserved5;
    }
    else {
      resolved.reserved5 = 0
    }

    return resolved;
    }
};

// Constants for message
CfgPRT.Constants = {
  CLASS_ID: 6,
  MESSAGE_ID: 0,
  PORT_ID_UART1: 1,
  PORT_ID_UART2: 2,
  PORT_ID_USB: 3,
  TX_READY_EN: 1,
  TX_READY_POLARITY_HIGH_ACTIVE: 0,
  TX_READY_POLARITY_LOW_ACTIVE: 2,
  TX_READY_PIN_SHIFT: 2,
  TX_READY_PIN_MASK: 124,
  TX_READY_THRES_SHIFT: 7,
  TX_READY_THRES_MASK: 65408,
  MODE_RESERVED1: 16,
  MODE_CHAR_LEN_MASK: 192,
  MODE_CHAR_LEN_5BIT: 0,
  MODE_CHAR_LEN_6BIT: 64,
  MODE_CHAR_LEN_7BIT: 128,
  MODE_CHAR_LEN_8BIT: 192,
  MODE_PARITY_MASK: 3584,
  MODE_PARITY_EVEN: 0,
  MODE_PARITY_ODD: 512,
  MODE_PARITY_NO: 2048,
  MODE_STOP_BITS_MASK: 12288,
  MODE_STOP_BITS_1: 0,
  MODE_STOP_BITS_15: 4096,
  MODE_STOP_BITS_2: 8192,
  MODE_STOP_BITS_05: 12288,
  PROTO_UBX: 1,
  PROTO_NMEA: 2,
  PROTO_RTCM: 4,
}

module.exports = CfgPRT;
