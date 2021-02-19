// Auto-generated. Do not edit!

// (in-package lka.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Lane = require('./Lane.js');

//-----------------------------------------------------------

class Lanes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lane_lines = null;
    }
    else {
      if (initObj.hasOwnProperty('lane_lines')) {
        this.lane_lines = initObj.lane_lines
      }
      else {
        this.lane_lines = new Array(2).fill(new Lane());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Lanes
    // Check that the constant length array field [lane_lines] has the right length
    if (obj.lane_lines.length !== 2) {
      throw new Error('Unable to serialize array field lane_lines - length must be 2')
    }
    // Serialize message field [lane_lines]
    obj.lane_lines.forEach((val) => {
      bufferOffset = Lane.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Lanes
    let len;
    let data = new Lanes(null);
    // Deserialize message field [lane_lines]
    len = 2;
    data.lane_lines = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.lane_lines[i] = Lane.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lka/Lanes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd937409978545c3e3e3fe66ef4b4e7c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Lane[2] lane_lines
    ================================================================================
    MSG: lka/Lane
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
    const resolved = new Lanes(null);
    if (msg.lane_lines !== undefined) {
      resolved.lane_lines = new Array(2)
      for (let i = 0; i < resolved.lane_lines.length; ++i) {
        if (msg.lane_lines.length > i) {
          resolved.lane_lines[i] = Lane.Resolve(msg.lane_lines[i]);
        }
        else {
          resolved.lane_lines[i] = new Lane();
        }
      }
    }
    else {
      resolved.lane_lines = new Array(2).fill(new Lane())
    }

    return resolved;
    }
};

module.exports = Lanes;
