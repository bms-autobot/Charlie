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

class CfgNAVX5 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.version = null;
      this.mask1 = null;
      this.reserved0 = null;
      this.reserved1 = null;
      this.reserved2 = null;
      this.minSVs = null;
      this.maxSVs = null;
      this.minCNO = null;
      this.reserved5 = null;
      this.iniFix3D = null;
      this.reserved6 = null;
      this.reserved7 = null;
      this.ackAiding = null;
      this.wknRollover = null;
      this.reserved8 = null;
      this.reserved9 = null;
      this.reserved10 = null;
      this.usePPP = null;
      this.aopCfg = null;
      this.reserved11 = null;
      this.reserved12 = null;
      this.aopOrbMaxErr = null;
      this.reserved13 = null;
      this.reserved14 = null;
      this.reserved3 = null;
      this.reserved4 = null;
    }
    else {
      if (initObj.hasOwnProperty('version')) {
        this.version = initObj.version
      }
      else {
        this.version = 0;
      }
      if (initObj.hasOwnProperty('mask1')) {
        this.mask1 = initObj.mask1
      }
      else {
        this.mask1 = 0;
      }
      if (initObj.hasOwnProperty('reserved0')) {
        this.reserved0 = initObj.reserved0
      }
      else {
        this.reserved0 = 0;
      }
      if (initObj.hasOwnProperty('reserved1')) {
        this.reserved1 = initObj.reserved1
      }
      else {
        this.reserved1 = 0;
      }
      if (initObj.hasOwnProperty('reserved2')) {
        this.reserved2 = initObj.reserved2
      }
      else {
        this.reserved2 = 0;
      }
      if (initObj.hasOwnProperty('minSVs')) {
        this.minSVs = initObj.minSVs
      }
      else {
        this.minSVs = 0;
      }
      if (initObj.hasOwnProperty('maxSVs')) {
        this.maxSVs = initObj.maxSVs
      }
      else {
        this.maxSVs = 0;
      }
      if (initObj.hasOwnProperty('minCNO')) {
        this.minCNO = initObj.minCNO
      }
      else {
        this.minCNO = 0;
      }
      if (initObj.hasOwnProperty('reserved5')) {
        this.reserved5 = initObj.reserved5
      }
      else {
        this.reserved5 = 0;
      }
      if (initObj.hasOwnProperty('iniFix3D')) {
        this.iniFix3D = initObj.iniFix3D
      }
      else {
        this.iniFix3D = 0;
      }
      if (initObj.hasOwnProperty('reserved6')) {
        this.reserved6 = initObj.reserved6
      }
      else {
        this.reserved6 = 0;
      }
      if (initObj.hasOwnProperty('reserved7')) {
        this.reserved7 = initObj.reserved7
      }
      else {
        this.reserved7 = 0;
      }
      if (initObj.hasOwnProperty('ackAiding')) {
        this.ackAiding = initObj.ackAiding
      }
      else {
        this.ackAiding = 0;
      }
      if (initObj.hasOwnProperty('wknRollover')) {
        this.wknRollover = initObj.wknRollover
      }
      else {
        this.wknRollover = 0;
      }
      if (initObj.hasOwnProperty('reserved8')) {
        this.reserved8 = initObj.reserved8
      }
      else {
        this.reserved8 = 0;
      }
      if (initObj.hasOwnProperty('reserved9')) {
        this.reserved9 = initObj.reserved9
      }
      else {
        this.reserved9 = 0;
      }
      if (initObj.hasOwnProperty('reserved10')) {
        this.reserved10 = initObj.reserved10
      }
      else {
        this.reserved10 = 0;
      }
      if (initObj.hasOwnProperty('usePPP')) {
        this.usePPP = initObj.usePPP
      }
      else {
        this.usePPP = 0;
      }
      if (initObj.hasOwnProperty('aopCfg')) {
        this.aopCfg = initObj.aopCfg
      }
      else {
        this.aopCfg = 0;
      }
      if (initObj.hasOwnProperty('reserved11')) {
        this.reserved11 = initObj.reserved11
      }
      else {
        this.reserved11 = 0;
      }
      if (initObj.hasOwnProperty('reserved12')) {
        this.reserved12 = initObj.reserved12
      }
      else {
        this.reserved12 = 0;
      }
      if (initObj.hasOwnProperty('aopOrbMaxErr')) {
        this.aopOrbMaxErr = initObj.aopOrbMaxErr
      }
      else {
        this.aopOrbMaxErr = 0;
      }
      if (initObj.hasOwnProperty('reserved13')) {
        this.reserved13 = initObj.reserved13
      }
      else {
        this.reserved13 = 0;
      }
      if (initObj.hasOwnProperty('reserved14')) {
        this.reserved14 = initObj.reserved14
      }
      else {
        this.reserved14 = 0;
      }
      if (initObj.hasOwnProperty('reserved3')) {
        this.reserved3 = initObj.reserved3
      }
      else {
        this.reserved3 = 0;
      }
      if (initObj.hasOwnProperty('reserved4')) {
        this.reserved4 = initObj.reserved4
      }
      else {
        this.reserved4 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CfgNAVX5
    // Serialize message field [version]
    bufferOffset = _serializer.uint16(obj.version, buffer, bufferOffset);
    // Serialize message field [mask1]
    bufferOffset = _serializer.uint16(obj.mask1, buffer, bufferOffset);
    // Serialize message field [reserved0]
    bufferOffset = _serializer.uint32(obj.reserved0, buffer, bufferOffset);
    // Serialize message field [reserved1]
    bufferOffset = _serializer.uint8(obj.reserved1, buffer, bufferOffset);
    // Serialize message field [reserved2]
    bufferOffset = _serializer.uint8(obj.reserved2, buffer, bufferOffset);
    // Serialize message field [minSVs]
    bufferOffset = _serializer.uint8(obj.minSVs, buffer, bufferOffset);
    // Serialize message field [maxSVs]
    bufferOffset = _serializer.uint8(obj.maxSVs, buffer, bufferOffset);
    // Serialize message field [minCNO]
    bufferOffset = _serializer.uint8(obj.minCNO, buffer, bufferOffset);
    // Serialize message field [reserved5]
    bufferOffset = _serializer.uint8(obj.reserved5, buffer, bufferOffset);
    // Serialize message field [iniFix3D]
    bufferOffset = _serializer.uint8(obj.iniFix3D, buffer, bufferOffset);
    // Serialize message field [reserved6]
    bufferOffset = _serializer.uint8(obj.reserved6, buffer, bufferOffset);
    // Serialize message field [reserved7]
    bufferOffset = _serializer.uint8(obj.reserved7, buffer, bufferOffset);
    // Serialize message field [ackAiding]
    bufferOffset = _serializer.uint8(obj.ackAiding, buffer, bufferOffset);
    // Serialize message field [wknRollover]
    bufferOffset = _serializer.uint16(obj.wknRollover, buffer, bufferOffset);
    // Serialize message field [reserved8]
    bufferOffset = _serializer.uint32(obj.reserved8, buffer, bufferOffset);
    // Serialize message field [reserved9]
    bufferOffset = _serializer.uint8(obj.reserved9, buffer, bufferOffset);
    // Serialize message field [reserved10]
    bufferOffset = _serializer.uint8(obj.reserved10, buffer, bufferOffset);
    // Serialize message field [usePPP]
    bufferOffset = _serializer.uint8(obj.usePPP, buffer, bufferOffset);
    // Serialize message field [aopCfg]
    bufferOffset = _serializer.uint8(obj.aopCfg, buffer, bufferOffset);
    // Serialize message field [reserved11]
    bufferOffset = _serializer.uint8(obj.reserved11, buffer, bufferOffset);
    // Serialize message field [reserved12]
    bufferOffset = _serializer.uint8(obj.reserved12, buffer, bufferOffset);
    // Serialize message field [aopOrbMaxErr]
    bufferOffset = _serializer.uint16(obj.aopOrbMaxErr, buffer, bufferOffset);
    // Serialize message field [reserved13]
    bufferOffset = _serializer.uint8(obj.reserved13, buffer, bufferOffset);
    // Serialize message field [reserved14]
    bufferOffset = _serializer.uint8(obj.reserved14, buffer, bufferOffset);
    // Serialize message field [reserved3]
    bufferOffset = _serializer.uint16(obj.reserved3, buffer, bufferOffset);
    // Serialize message field [reserved4]
    bufferOffset = _serializer.uint32(obj.reserved4, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CfgNAVX5
    let len;
    let data = new CfgNAVX5(null);
    // Deserialize message field [version]
    data.version = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [mask1]
    data.mask1 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [reserved0]
    data.reserved0 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [reserved1]
    data.reserved1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved2]
    data.reserved2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [minSVs]
    data.minSVs = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [maxSVs]
    data.maxSVs = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [minCNO]
    data.minCNO = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved5]
    data.reserved5 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [iniFix3D]
    data.iniFix3D = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved6]
    data.reserved6 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved7]
    data.reserved7 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ackAiding]
    data.ackAiding = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [wknRollover]
    data.wknRollover = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [reserved8]
    data.reserved8 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [reserved9]
    data.reserved9 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved10]
    data.reserved10 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [usePPP]
    data.usePPP = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [aopCfg]
    data.aopCfg = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved11]
    data.reserved11 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved12]
    data.reserved12 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [aopOrbMaxErr]
    data.aopOrbMaxErr = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [reserved13]
    data.reserved13 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved14]
    data.reserved14 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved3]
    data.reserved3 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [reserved4]
    data.reserved4 = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ublox_msgs/CfgNAVX5';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51508a96b05799aa4924b225bcb10fb3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # CFG-NAVX5 (0x06 0x24)
    # Navigation Engine Expert Settings
    # Warning: Refer to u-blox protocol spec before changing these settings.
    
    uint8 CLASS_ID = 6
    uint8 MESSAGE_ID = 35
    
    uint16 version        # Message version (set to 0)
    
    uint16 mask1          # First parameters bitmask (possible values below)
    uint16 MASK_MIN_MAX        = 4
    uint16 MASK_MIN_CNO        = 8
    uint16 MASK_INITIAL_FIX_3D = 64
    uint16 MASK_WKN_ROLL       = 512
    uint16 MASK_PPP            = 8192
    uint16 MASK_AOP            = 16384
    
    uint32 reserved0      # Always set to zero
    uint8 reserved1       # Always set to zero
    uint8 reserved2       # Always set to zero
    
    uint8 minSVs          # Minimum number of satellites for navigation
    uint8 maxSVs          # Maximum number of satellites for navigation
    uint8 minCNO          # Minimum satellite signal level for navigation
    
    uint8 reserved5       # Always set to zero
    
    uint8 iniFix3D        # If set to 1, initial fix must be 3D
    
    uint8 reserved6       # Always set to zero
    uint8 reserved7       # Always set to zero
    
    uint8 ackAiding       # If set to 1, issue acknowledgements for assistance
    uint16 wknRollover    # GPS rollover number
    
    uint32 reserved8      # Always set to zero
    uint8 reserved9       # Always set to zero
    uint8 reserved10      # Always set to zero
    
    uint8 usePPP          # Enable/disable PPP (on supported units)
    uint8 aopCfg          # AssistNow Autonomous configuration
    
    uint8 reserved11      # Always set to zero
    uint8 reserved12      # Always set to zero
    
    uint16 aopOrbMaxErr   # Maximum acceptable (modelled) autonomous orbit error
    
    uint8 reserved13      # Always set to zero
    uint8 reserved14      # Always set to zero
    uint16 reserved3      # Always set to zero
    uint32 reserved4      # Always set to zero
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CfgNAVX5(null);
    if (msg.version !== undefined) {
      resolved.version = msg.version;
    }
    else {
      resolved.version = 0
    }

    if (msg.mask1 !== undefined) {
      resolved.mask1 = msg.mask1;
    }
    else {
      resolved.mask1 = 0
    }

    if (msg.reserved0 !== undefined) {
      resolved.reserved0 = msg.reserved0;
    }
    else {
      resolved.reserved0 = 0
    }

    if (msg.reserved1 !== undefined) {
      resolved.reserved1 = msg.reserved1;
    }
    else {
      resolved.reserved1 = 0
    }

    if (msg.reserved2 !== undefined) {
      resolved.reserved2 = msg.reserved2;
    }
    else {
      resolved.reserved2 = 0
    }

    if (msg.minSVs !== undefined) {
      resolved.minSVs = msg.minSVs;
    }
    else {
      resolved.minSVs = 0
    }

    if (msg.maxSVs !== undefined) {
      resolved.maxSVs = msg.maxSVs;
    }
    else {
      resolved.maxSVs = 0
    }

    if (msg.minCNO !== undefined) {
      resolved.minCNO = msg.minCNO;
    }
    else {
      resolved.minCNO = 0
    }

    if (msg.reserved5 !== undefined) {
      resolved.reserved5 = msg.reserved5;
    }
    else {
      resolved.reserved5 = 0
    }

    if (msg.iniFix3D !== undefined) {
      resolved.iniFix3D = msg.iniFix3D;
    }
    else {
      resolved.iniFix3D = 0
    }

    if (msg.reserved6 !== undefined) {
      resolved.reserved6 = msg.reserved6;
    }
    else {
      resolved.reserved6 = 0
    }

    if (msg.reserved7 !== undefined) {
      resolved.reserved7 = msg.reserved7;
    }
    else {
      resolved.reserved7 = 0
    }

    if (msg.ackAiding !== undefined) {
      resolved.ackAiding = msg.ackAiding;
    }
    else {
      resolved.ackAiding = 0
    }

    if (msg.wknRollover !== undefined) {
      resolved.wknRollover = msg.wknRollover;
    }
    else {
      resolved.wknRollover = 0
    }

    if (msg.reserved8 !== undefined) {
      resolved.reserved8 = msg.reserved8;
    }
    else {
      resolved.reserved8 = 0
    }

    if (msg.reserved9 !== undefined) {
      resolved.reserved9 = msg.reserved9;
    }
    else {
      resolved.reserved9 = 0
    }

    if (msg.reserved10 !== undefined) {
      resolved.reserved10 = msg.reserved10;
    }
    else {
      resolved.reserved10 = 0
    }

    if (msg.usePPP !== undefined) {
      resolved.usePPP = msg.usePPP;
    }
    else {
      resolved.usePPP = 0
    }

    if (msg.aopCfg !== undefined) {
      resolved.aopCfg = msg.aopCfg;
    }
    else {
      resolved.aopCfg = 0
    }

    if (msg.reserved11 !== undefined) {
      resolved.reserved11 = msg.reserved11;
    }
    else {
      resolved.reserved11 = 0
    }

    if (msg.reserved12 !== undefined) {
      resolved.reserved12 = msg.reserved12;
    }
    else {
      resolved.reserved12 = 0
    }

    if (msg.aopOrbMaxErr !== undefined) {
      resolved.aopOrbMaxErr = msg.aopOrbMaxErr;
    }
    else {
      resolved.aopOrbMaxErr = 0
    }

    if (msg.reserved13 !== undefined) {
      resolved.reserved13 = msg.reserved13;
    }
    else {
      resolved.reserved13 = 0
    }

    if (msg.reserved14 !== undefined) {
      resolved.reserved14 = msg.reserved14;
    }
    else {
      resolved.reserved14 = 0
    }

    if (msg.reserved3 !== undefined) {
      resolved.reserved3 = msg.reserved3;
    }
    else {
      resolved.reserved3 = 0
    }

    if (msg.reserved4 !== undefined) {
      resolved.reserved4 = msg.reserved4;
    }
    else {
      resolved.reserved4 = 0
    }

    return resolved;
    }
};

// Constants for message
CfgNAVX5.Constants = {
  CLASS_ID: 6,
  MESSAGE_ID: 35,
  MASK_MIN_MAX: 4,
  MASK_MIN_CNO: 8,
  MASK_INITIAL_FIX_3D: 64,
  MASK_WKN_ROLL: 512,
  MASK_PPP: 8192,
  MASK_AOP: 16384,
}

module.exports = CfgNAVX5;
