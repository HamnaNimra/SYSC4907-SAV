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

class Margins {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.margin_diff = null;
    }
    else {
      if (initObj.hasOwnProperty('margin_diff')) {
        this.margin_diff = initObj.margin_diff
      }
      else {
        this.margin_diff = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Margins
    // Serialize message field [margin_diff]
    bufferOffset = _serializer.float64(obj.margin_diff, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Margins
    let len;
    let data = new Margins(null);
    // Deserialize message field [margin_diff]
    data.margin_diff = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lka/Margins';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8a2be87eb93522a3eb44d383e70dffc2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 margin_diff
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Margins(null);
    if (msg.margin_diff !== undefined) {
      resolved.margin_diff = msg.margin_diff;
    }
    else {
      resolved.margin_diff = 0.0
    }

    return resolved;
    }
};

module.exports = Margins;
