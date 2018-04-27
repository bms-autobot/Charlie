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

class CfgGNSS {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.msgVer = null;
      this.numTrkChHw = null;
      this.numTrkChUse = null;
      this.numConfigBlocks = null;
      this.gnssId = null;
      this.resTrkCh = null;
      this.maxTrkCh = null;
      this.reserved1 = null;
      this.flags = null;
    }
    else {
      if (initObj.hasOwnProperty('msgVer')) {
        this.msgVer = initObj.msgVer
      }
      else {
        this.msgVer = 0;
      }
      if (initObj.hasOwnProperty('numTrkChHw')) {
        this.numTrkChHw = initObj.numTrkChHw
      }
      else {
        this.numTrkChHw = 0;
      }
      if (initObj.hasOwnProperty('numTrkChUse')) {
        this.numTrkChUse = initObj.numTrkChUse
      }
      else {
        this.numTrkChUse = 0;
      }
      if (initObj.hasOwnProperty('numConfigBlocks')) {
        this.numConfigBlocks = initObj.numConfigBlocks
      }
      else {
        this.numConfigBlocks = 0;
      }
      if (initObj.hasOwnProperty('gnssId')) {
        this.gnssId = initObj.gnssId
      }
      else {
        this.gnssId = 0;
      }
      if (initObj.hasOwnProperty('resTrkCh')) {
        this.resTrkCh = initObj.resTrkCh
      }
      else {
        this.resTrkCh = 0;
      }
      if (initObj.hasOwnProperty('maxTrkCh')) {
        this.maxTrkCh = initObj.maxTrkCh
      }
      else {
        this.maxTrkCh = 0;
      }
      if (initObj.hasOwnProperty('reserved1')) {
        this.reserved1 = initObj.reserved1
      }
      else {
        this.reserved1 = 0;
      }
      if (initObj.hasOwnProperty('flags')) {
        this.flags = initObj.flags
      }
      else {
        this.flags = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CfgGNSS
    // Serialize message field [msgVer]
    bufferOffset = _serializer.uint8(obj.msgVer, buffer, bufferOffset);
    // Serialize message field [numTrkChHw]
    bufferOffset = _serializer.uint8(obj.numTrkChHw, buffer, bufferOffset);
    // Serialize message field [numTrkChUse]
    bufferOffset = _serializer.uint8(obj.numTrkChUse, buffer, bufferOffset);
    // Serialize message field [numConfigBlocks]
    bufferOffset = _serializer.uint8(obj.numConfigBlocks, buffer, bufferOffset);
    // Serialize message field [gnssId]
    bufferOffset = _serializer.uint8(obj.gnssId, buffer, bufferOffset);
    // Serialize message field [resTrkCh]
    bufferOffset = _serializer.uint8(obj.resTrkCh, buffer, bufferOffset);
    // Serialize message field [maxTrkCh]
    bufferOffset = _serializer.uint8(obj.maxTrkCh, buffer, bufferOffset);
    // Serialize message field [reserved1]
    bufferOffset = _serializer.uint8(obj.reserved1, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CfgGNSS
    let len;
    let data = new CfgGNSS(null);
    // Deserialize message field [msgVer]
    data.msgVer = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [numTrkChHw]
    data.numTrkChHw = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [numTrkChUse]
    data.numTrkChUse = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [numConfigBlocks]
    data.numConfigBlocks = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gnssId]
    data.gnssId = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [resTrkCh]
    data.resTrkCh = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [maxTrkCh]
    data.maxTrkCh = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reserved1]
    data.reserved1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ublox_msgs/CfgGNSS';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b7f92af0b44aa7c047fd14c5926fb0fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # CFG-GNSS (0x06 0x3E)
    # GNSS Configuration
    # Gets or sets the GNSS system channel sharing configuration.
    
    uint8 CLASS_ID = 6
    uint8 MESSAGE_ID = 62
    
    uint8 msgVer            # Message version (= 0 for this version)
    uint8 numTrkChHw        # Number of tracking channels in hardware (read only)
    uint8 numTrkChUse       # Number of tracking channels to use (<= numTrkChHw)
    uint8 numConfigBlocks   # Number of configuration blocks following (set to 1)
    
    uint8 gnssId            # GNSS identifier (select from following list)
    
    uint8 GNSS_ID_GPS = 0
    uint8 GNSS_ID_SBAS = 1
    uint8 GNSS_ID_GALILEO = 2
    uint8 GNSS_ID_BEIDOU = 3
    uint8 GNSS_ID_QZSS = 5
    uint8 GNSS_ID_GLONASS = 6
    
    uint8 resTrkCh          # Number of reserved (minimum) tracking channels for this GNSS system
    uint8 maxTrkCh          # Maximum number of tracking channels (>=resTrkChn)
    uint8 reserved1         # Reserved
    uint32 flags            # Bitfield of flags (bit 0 = enable/disable)
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CfgGNSS(null);
    if (msg.msgVer !== undefined) {
      resolved.msgVer = msg.msgVer;
    }
    else {
      resolved.msgVer = 0
    }

    if (msg.numTrkChHw !== undefined) {
      resolved.numTrkChHw = msg.numTrkChHw;
    }
    else {
      resolved.numTrkChHw = 0
    }

    if (msg.numTrkChUse !== undefined) {
      resolved.numTrkChUse = msg.numTrkChUse;
    }
    else {
      resolved.numTrkChUse = 0
    }

    if (msg.numConfigBlocks !== undefined) {
      resolved.numConfigBlocks = msg.numConfigBlocks;
    }
    else {
      resolved.numConfigBlocks = 0
    }

    if (msg.gnssId !== undefined) {
      resolved.gnssId = msg.gnssId;
    }
    else {
      resolved.gnssId = 0
    }

    if (msg.resTrkCh !== undefined) {
      resolved.resTrkCh = msg.resTrkCh;
    }
    else {
      resolved.resTrkCh = 0
    }

    if (msg.maxTrkCh !== undefined) {
      resolved.maxTrkCh = msg.maxTrkCh;
    }
    else {
      resolved.maxTrkCh = 0
    }

    if (msg.reserved1 !== undefined) {
      resolved.reserved1 = msg.reserved1;
    }
    else {
      resolved.reserved1 = 0
    }

    if (msg.flags !== undefined) {
      resolved.flags = msg.flags;
    }
    else {
      resolved.flags = 0
    }

    return resolved;
    }
};

// Constants for message
CfgGNSS.Constants = {
  CLASS_ID: 6,
  MESSAGE_ID: 62,
  GNSS_ID_GPS: 0,
  GNSS_ID_SBAS: 1,
  GNSS_ID_GALILEO: 2,
  GNSS_ID_BEIDOU: 3,
  GNSS_ID_QZSS: 5,
  GNSS_ID_GLONASS: 6,
}

module.exports = CfgGNSS;
