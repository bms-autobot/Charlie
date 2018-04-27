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

class CfgNAV5 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mask = null;
      this.dynModel = null;
      this.fixMode = null;
      this.fixedAlt = null;
      this.fixedAltVar = null;
      this.minElev = null;
      this.drLimit = null;
      this.pDop = null;
      this.tDop = null;
      this.pAcc = null;
      this.tAcc = null;
      this.staticHoldThresh = null;
      this.dgpsTimeOut = null;
      this.reserved2 = null;
      this.reserved3 = null;
      this.reserved4 = null;
    }
    else {
      if (initObj.hasOwnProperty('mask')) {
        this.mask = initObj.mask
      }
      else {
        this.mask = 0;
      }
      if (initObj.hasOwnProperty('dynModel')) {
        this.dynModel = initObj.dynModel
      }
      else {
        this.dynModel = 0;
      }
      if (initObj.hasOwnProperty('fixMode')) {
        this.fixMode = initObj.fixMode
      }
      else {
        this.fixMode = 0;
      }
      if (initObj.hasOwnProperty('fixedAlt')) {
        this.fixedAlt = initObj.fixedAlt
      }
      else {
        this.fixedAlt = 0;
      }
      if (initObj.hasOwnProperty('fixedAltVar')) {
        this.fixedAltVar = initObj.fixedAltVar
      }
      else {
        this.fixedAltVar = 0;
      }
      if (initObj.hasOwnProperty('minElev')) {
        this.minElev = initObj.minElev
      }
      else {
        this.minElev = 0;
      }
      if (initObj.hasOwnProperty('drLimit')) {
        this.drLimit = initObj.drLimit
      }
      else {
        this.drLimit = 0;
      }
      if (initObj.hasOwnProperty('pDop')) {
        this.pDop = initObj.pDop
      }
      else {
        this.pDop = 0;
      }
      if (initObj.hasOwnProperty('tDop')) {
        this.tDop = initObj.tDop
      }
      else {
        this.tDop = 0;
      }
      if (initObj.hasOwnProperty('pAcc')) {
        this.pAcc = initObj.pAcc
      }
      else {
        this.pAcc = 0;
      }
      if (initObj.hasOwnProperty('tAcc')) {
        this.tAcc = initObj.tAcc
      }
      else {
        this.tAcc = 0;
      }
      if (initObj.hasOwnProperty('staticHoldThresh')) {
        this.staticHoldThresh = initObj.staticHoldThresh
      }
      else {
        this.staticHoldThresh = 0;
      }
      if (initObj.hasOwnProperty('dgpsTimeOut')) {
        this.dgpsTimeOut = initObj.dgpsTimeOut
      }
      else {
        this.dgpsTimeOut = 0;
      }
      if (initObj.hasOwnProperty('reserved2')) {
        this.reserved2 = initObj.reserved2
      }
      else {
        this.reserved2 = 0;
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
    // Serializes a message object of type CfgNAV5
    // Serialize message field [mask]
    bufferOffset = _serializer.uint16(obj.mask, buffer, bufferOffset);
    // Serialize message field [dynModel]
    bufferOffset = _serializer.uint8(obj.dynModel, buffer, bufferOffset);
    // Serialize message field [fixMode]
    bufferOffset = _serializer.uint8(obj.fixMode, buffer, bufferOffset);
    // Serialize message field [fixedAlt]
    bufferOffset = _serializer.int32(obj.fixedAlt, buffer, bufferOffset);
    // Serialize message field [fixedAltVar]
    bufferOffset = _serializer.uint32(obj.fixedAltVar, buffer, bufferOffset);
    // Serialize message field [minElev]
    bufferOffset = _serializer.int8(obj.minElev, buffer, bufferOffset);
    // Serialize message field [drLimit]
    bufferOffset = _serializer.uint8(obj.drLimit, buffer, bufferOffset);
    // Serialize message field [pDop]
    bufferOffset = _serializer.uint16(obj.pDop, buffer, bufferOffset);
    // Serialize message field [tDop]
    bufferOffset = _serializer.uint16(obj.tDop, buffer, bufferOffset);
    // Serialize message field [pAcc]
    bufferOffset = _serializer.uint16(obj.pAcc, buffer, bufferOffset);
    // Serialize message field [tAcc]
    bufferOffset = _serializer.uint16(obj.tAcc, buffer, bufferOffset);
    // Serialize message field [staticHoldThresh]
    bufferOffset = _serializer.uint8(obj.staticHoldThresh, buffer, bufferOffset);
    // Serialize message field [dgpsTimeOut]
    bufferOffset = _serializer.uint8(obj.dgpsTimeOut, buffer, bufferOffset);
    // Serialize message field [reserved2]
    bufferOffset = _serializer.uint32(obj.reserved2, buffer, bufferOffset);
    // Serialize message field [reserved3]
    bufferOffset = _serializer.uint32(obj.reserved3, buffer, bufferOffset);
    // Serialize message field [reserved4]
    bufferOffset = _serializer.uint32(obj.reserved4, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CfgNAV5
    let len;
    let data = new CfgNAV5(null);
    // Deserialize message field [mask]
    data.mask = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [dynModel]
    data.dynModel = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [fixMode]
    data.fixMode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [fixedAlt]
    data.fixedAlt = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [fixedAltVar]
    data.fixedAltVar = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [minElev]
    data.minElev = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [drLimit]
    data.drLimit = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pDop]
    data.pDop = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tDop]
    data.tDop = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [pAcc]
    data.pAcc = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tAcc]
    data.tAcc = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [staticHoldThresh]
    data.staticHoldThresh = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dgpsTimeOut]
    data.dgpsTimeOut = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved2]
    data.reserved2 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [reserved3]
    data.reserved3 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [reserved4]
    data.reserved4 = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ublox_msgs/CfgNAV5';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c6e2419d1dd6532b8f09e39a02e167c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # CFG-NAV5 (0x06 0x24)
    # Navigation Engine Settings
    #
    
    uint8 CLASS_ID = 6
    uint8 MESSAGE_ID = 36
    
    uint16 mask             # Parameters Bitmask. Only the masked
                            # parameters will be applied.
    uint16 MASK_DYN = 1                 # Apply dynamic model settings
    uint16 MASK_MIN_EL = 2              # Apply minimum elevation settings
    uint16 MASK_FIX_MODE = 4            # Apply fix mode settings
    uint16 MASK_DR_LIM = 8              # Apply DR limit settings
    uint16 MASK_POS_MASK = 16           # Apply position mask settings
    uint16 MASK_TIME_MASK = 32          # Apply time mask settings
    uint16 MASK_STATIC_HOLD_MASK = 64   # Apply static hold settings
    uint16 MASK_DGPS_MASK = 128         # Apply DGPS settings, firmware 7 and newer only
    
    uint8 dynModel          # Dynamic Platform model:
    uint8 DYN_MODEL_PORTABLE = 0        # Portable
    uint8 DYN_MODEL_STATIONARY = 2      # Stationary
    uint8 DYN_MODEL_PEDESTRIAN = 3      # Pedestrian
    uint8 DYN_MODEL_AUTOMOTIVE = 4      # Automotive
    uint8 DYN_MODEL_SEA = 5             # Sea
    uint8 DYN_MODEL_AIRBORNE_1G = 6     # Airborne with <1g Acceleration
    uint8 DYN_MODEL_AIRBORNE_2G = 7     # Airborne with <2g Acceleration
    uint8 DYN_MODEL_AIRBORNE_4G = 8     # Airborne with <4g Acceleration
    
    uint8 fixMode           # Position Fixing Mode.
    uint8 FIX_MODE_2D_ONLY = 1          # 2D only
    uint8 FIX_MODE_3D_ONLY = 2          # 3D only
    uint8 FIX_MODE_AUTO = 3             # Auto 2D/3D
    
    int32 fixedAlt          # Fixed altitude (mean sea level) for 2D fix mode. [m / 0.01]
    uint32 fixedAltVar      # Fixed altitude variance for 2D mode. [m^2 / 0.0001]
    int8 minElev            # Minimum Elevation for a GNSS satellite to be used in NAV [deg]
    uint8 drLimit           # Maximum time to perform dead reckoning [s]
                            # (linear extrapolation) in case of GPS signal loss
    uint16 pDop             # Position DOP Mask to use [1 / 0.1]
    uint16 tDop             # Time DOP Mask to use [1 / 0.1]
    uint16 pAcc             # Position Accuracy Mask [m]
    uint16 tAcc             # Time Accuracy Mask [m]
    uint8 staticHoldThresh  # Static hold threshold [cm/s]
    uint8 dgpsTimeOut       # DGPS timeout, firmware 7 and newer only [s]
    uint32 reserved2        # Always set to zero
    uint32 reserved3        # Always set to zero
    uint32 reserved4        # Always set to zero
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CfgNAV5(null);
    if (msg.mask !== undefined) {
      resolved.mask = msg.mask;
    }
    else {
      resolved.mask = 0
    }

    if (msg.dynModel !== undefined) {
      resolved.dynModel = msg.dynModel;
    }
    else {
      resolved.dynModel = 0
    }

    if (msg.fixMode !== undefined) {
      resolved.fixMode = msg.fixMode;
    }
    else {
      resolved.fixMode = 0
    }

    if (msg.fixedAlt !== undefined) {
      resolved.fixedAlt = msg.fixedAlt;
    }
    else {
      resolved.fixedAlt = 0
    }

    if (msg.fixedAltVar !== undefined) {
      resolved.fixedAltVar = msg.fixedAltVar;
    }
    else {
      resolved.fixedAltVar = 0
    }

    if (msg.minElev !== undefined) {
      resolved.minElev = msg.minElev;
    }
    else {
      resolved.minElev = 0
    }

    if (msg.drLimit !== undefined) {
      resolved.drLimit = msg.drLimit;
    }
    else {
      resolved.drLimit = 0
    }

    if (msg.pDop !== undefined) {
      resolved.pDop = msg.pDop;
    }
    else {
      resolved.pDop = 0
    }

    if (msg.tDop !== undefined) {
      resolved.tDop = msg.tDop;
    }
    else {
      resolved.tDop = 0
    }

    if (msg.pAcc !== undefined) {
      resolved.pAcc = msg.pAcc;
    }
    else {
      resolved.pAcc = 0
    }

    if (msg.tAcc !== undefined) {
      resolved.tAcc = msg.tAcc;
    }
    else {
      resolved.tAcc = 0
    }

    if (msg.staticHoldThresh !== undefined) {
      resolved.staticHoldThresh = msg.staticHoldThresh;
    }
    else {
      resolved.staticHoldThresh = 0
    }

    if (msg.dgpsTimeOut !== undefined) {
      resolved.dgpsTimeOut = msg.dgpsTimeOut;
    }
    else {
      resolved.dgpsTimeOut = 0
    }

    if (msg.reserved2 !== undefined) {
      resolved.reserved2 = msg.reserved2;
    }
    else {
      resolved.reserved2 = 0
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
CfgNAV5.Constants = {
  CLASS_ID: 6,
  MESSAGE_ID: 36,
  MASK_DYN: 1,
  MASK_MIN_EL: 2,
  MASK_FIX_MODE: 4,
  MASK_DR_LIM: 8,
  MASK_POS_MASK: 16,
  MASK_TIME_MASK: 32,
  MASK_STATIC_HOLD_MASK: 64,
  MASK_DGPS_MASK: 128,
  DYN_MODEL_PORTABLE: 0,
  DYN_MODEL_STATIONARY: 2,
  DYN_MODEL_PEDESTRIAN: 3,
  DYN_MODEL_AUTOMOTIVE: 4,
  DYN_MODEL_SEA: 5,
  DYN_MODEL_AIRBORNE_1G: 6,
  DYN_MODEL_AIRBORNE_2G: 7,
  DYN_MODEL_AIRBORNE_4G: 8,
  FIX_MODE_2D_ONLY: 1,
  FIX_MODE_3D_ONLY: 2,
  FIX_MODE_AUTO: 3,
}

module.exports = CfgNAV5;
