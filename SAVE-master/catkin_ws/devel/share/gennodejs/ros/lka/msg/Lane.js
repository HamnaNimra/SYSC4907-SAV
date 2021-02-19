// Auto-generated. Do not edit!

// (in-package lka.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Lane {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.exists = null;
      this.slope = null;
      this.y_cept = null;
    }
    else {
      if (initObj.hasOwnProperty('exists')) {
        this.exists = initObj.exists
      }
      else {
        this.exists = false;
      }
      if (initObj.hasOwnProperty('slope')) {
        this.slope = initObj.slope
      }
      else {
        this.slope = 0.0;
      }
      if (initObj.hasOwnProperty('y_cept')) {
        this.y_cept = initObj.y_cept
      }
      else {
        this.y_cept = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lane
    // Serialize message field [exists]
    bufferOffset = _serializer.bool(obj.exists, buffer, bufferOffset);
    // Serialize message field [slope]
    bufferOffset = _serializer.float64(obj.slope, buffer, bufferOffset);
    // Serialize message field [y_cept]
    bufferOffset = _serializer.float64(obj.y_cept, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lane
    let len;
    let data = new Lane(null);
    // Deserialize message field [exists]
    data.exists = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [slope]
    data.slope = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_cept]
    data.y_cept = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lka/Lane';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8fffbc133615afc331c7af31fc633d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool exists
    float64 slope
    float64 y_cept
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Lane(null);
    if (msg.exists !== undefined) {
      resolved.exists = msg.exists;
    }
    else {
      resolved.exists = false
    }

    if (msg.slope !== undefined) {
      resolved.slope = msg.slope;
    }
    else {
      resolved.slope = 0.0
    }

    if (msg.y_cept !== undefined) {
      resolved.y_cept = msg.y_cept;
    }
    else {
      resolved.y_cept = 0.0
    }

    return resolved;
    }
};

module.exports = Lane;
