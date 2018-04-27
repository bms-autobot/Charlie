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

class CfgRATE {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.measRate = null;
      this.navRate = null;
      this.timeRef = null;
    }
    else {
      if (initObj.hasOwnProperty('measRate')) {
        this.measRate = initObj.measRate
      }
      else {
        this.measRate = 0;
      }
      if (initObj.hasOwnProperty('navRate')) {
        this.navRate = initObj.navRate
      }
      else {
        this.navRate = 0;
      }
      if (initObj.hasOwnProperty('timeRef')) {
        this.timeRef = initObj.timeRef
      }
      else {
        this.timeRef = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CfgRATE
    // Serialize message field [measRate]
    bufferOffset = _serializer.uint16(obj.measRate, buffer, bufferOffset);
    // Serialize message field [navRate]
    bufferOffset = _serializer.uint16(obj.navRate, buffer, bufferOffset);
    // Serialize message field [timeRef]
    bufferOffset = _serializer.uint16(obj.timeRef, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CfgRATE
    let len;
    let data = new CfgRATE(null);
    // Deserialize message field [measRate]
    data.measRate = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [navRate]
    data.navRate = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [timeRef]
    data.timeRef = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ublox_msgs/CfgRATE';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da095554bde2600fd2a774def27fb48e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # CFG-RATE (0x06 0x08)
    # Navigation/Measurement Rate Settings
    #
    
    uint8 CLASS_ID = 6
    uint8 MESSAGE_ID = 8
    
    uint16 measRate           # Measurement Rate, GPS measurements are
                              # taken every measRate milliseconds [ms]
    uint16 navRate            # Navigation Rate, in number of measurement
                              # cycles. On u-blox 5 and u-blox 6, this parameter
                              # cannot be changed, and is always equals 1.
    uint16 timeRef            # Alignment to reference time: 0 = UTC time, 1 = GPS time
    uint16 TIME_REF_UTC = 0
    uint16 TIME_REF_GPS = 1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CfgRATE(null);
    if (msg.measRate !== undefined) {
      resolved.measRate = msg.measRate;
    }
    else {
      resolved.measRate = 0
    }

    if (msg.navRate !== undefined) {
      resolved.navRate = msg.navRate;
    }
    else {
      resolved.navRate = 0
    }

    if (msg.timeRef !== undefined) {
      resolved.timeRef = msg.timeRef;
    }
    else {
      resolved.timeRef = 0
    }

    return resolved;
    }
};

// Constants for message
CfgRATE.Constants = {
  CLASS_ID: 6,
  MESSAGE_ID: 8,
  TIME_REF_UTC: 0,
  TIME_REF_GPS: 1,
}

module.exports = CfgRATE;
