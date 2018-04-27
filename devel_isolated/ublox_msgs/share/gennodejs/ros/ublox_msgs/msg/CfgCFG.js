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

class CfgCFG {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.clearMask = null;
      this.saveMask = null;
      this.loadMask = null;
      this.deviceMask = null;
    }
    else {
      if (initObj.hasOwnProperty('clearMask')) {
        this.clearMask = initObj.clearMask
      }
      else {
        this.clearMask = 0;
      }
      if (initObj.hasOwnProperty('saveMask')) {
        this.saveMask = initObj.saveMask
      }
      else {
        this.saveMask = 0;
      }
      if (initObj.hasOwnProperty('loadMask')) {
        this.loadMask = initObj.loadMask
      }
      else {
        this.loadMask = 0;
      }
      if (initObj.hasOwnProperty('deviceMask')) {
        this.deviceMask = initObj.deviceMask
      }
      else {
        this.deviceMask = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CfgCFG
    // Serialize message field [clearMask]
    bufferOffset = _serializer.uint32(obj.clearMask, buffer, bufferOffset);
    // Serialize message field [saveMask]
    bufferOffset = _serializer.uint32(obj.saveMask, buffer, bufferOffset);
    // Serialize message field [loadMask]
    bufferOffset = _serializer.uint32(obj.loadMask, buffer, bufferOffset);
    // Serialize message field [deviceMask]
    bufferOffset = _serializer.uint8(obj.deviceMask, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CfgCFG
    let len;
    let data = new CfgCFG(null);
    // Deserialize message field [clearMask]
    data.clearMask = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [saveMask]
    data.saveMask = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [loadMask]
    data.loadMask = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [deviceMask]
    data.deviceMask = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ublox_msgs/CfgCFG';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '13fb5dc245f45e15f71ae45386c6e5b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # CFG-CFG (0x06 0x09)
    # Antenna Control Settings
    #
    
    uint8 CLASS_ID = 6
    uint8 MESSAGE_ID = 9
    
    uint32 clearMask          # Mask with configuration sub-sections to Clear
                              # (=Load Default Configurations to Permanent
                              # Configurations in non-volatile memory)
    uint32 saveMask           # Mask with configuration sub-section to Save
                              # (=Save Current Configuration to Non-volatile
                              # Memory)
    uint32 loadMask           # Mask with configuration sub-sections to Load
                              # (=Load Permanent Configurations from
                              # Non-volatile Memory to Current Configurations)
    
    uint32 MASK_IO_PORT = 1       # Port Settings
    uint32 MASK_MSG_CONF = 2      # Message Configuration
    uint32 MASK_INF_MSG = 4       # INF Message Configuration
    uint32 MASK_NAV_CONF = 8      # Navigation Configuration
    uint32 MASK_RXM_CONF = 16     # Receiver Manager Configuration
    uint32 MASK_RINV_CONF = 512   # Remote Inventory Configuration
    uint32 MASK_ANT_CONF = 1024   # Antenna Configuration
    
    uint8 deviceMask          # Mask which selects the devices for this command
    uint8 DEV_BBR = 1             # device battery backed RAM
    uint8 DEV_FLASH = 2           # device Flash
    uint8 DEV_EEPROM = 4          # device EEPROM
    uint8 DEV_SPI_FLASH = 16      # device SPI Flash
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CfgCFG(null);
    if (msg.clearMask !== undefined) {
      resolved.clearMask = msg.clearMask;
    }
    else {
      resolved.clearMask = 0
    }

    if (msg.saveMask !== undefined) {
      resolved.saveMask = msg.saveMask;
    }
    else {
      resolved.saveMask = 0
    }

    if (msg.loadMask !== undefined) {
      resolved.loadMask = msg.loadMask;
    }
    else {
      resolved.loadMask = 0
    }

    if (msg.deviceMask !== undefined) {
      resolved.deviceMask = msg.deviceMask;
    }
    else {
      resolved.deviceMask = 0
    }

    return resolved;
    }
};

// Constants for message
CfgCFG.Constants = {
  CLASS_ID: 6,
  MESSAGE_ID: 9,
  MASK_IO_PORT: 1,
  MASK_MSG_CONF: 2,
  MASK_INF_MSG: 4,
  MASK_NAV_CONF: 8,
  MASK_RXM_CONF: 16,
  MASK_RINV_CONF: 512,
  MASK_ANT_CONF: 1024,
  DEV_BBR: 1,
  DEV_FLASH: 2,
  DEV_EEPROM: 4,
  DEV_SPI_FLASH: 16,
}

module.exports = CfgCFG;
