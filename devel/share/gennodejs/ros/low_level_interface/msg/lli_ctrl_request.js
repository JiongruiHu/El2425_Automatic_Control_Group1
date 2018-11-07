// Auto-generated. Do not edit!

// (in-package low_level_interface.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class lli_ctrl_request {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.steering = null;
      this.velocity = null;
      this.transmission = null;
      this.differential_front = null;
      this.differential_rear = null;
      this.ctrl_code = null;
    }
    else {
      if (initObj.hasOwnProperty('steering')) {
        this.steering = initObj.steering
      }
      else {
        this.steering = 0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0;
      }
      if (initObj.hasOwnProperty('transmission')) {
        this.transmission = initObj.transmission
      }
      else {
        this.transmission = 0;
      }
      if (initObj.hasOwnProperty('differential_front')) {
        this.differential_front = initObj.differential_front
      }
      else {
        this.differential_front = 0;
      }
      if (initObj.hasOwnProperty('differential_rear')) {
        this.differential_rear = initObj.differential_rear
      }
      else {
        this.differential_rear = 0;
      }
      if (initObj.hasOwnProperty('ctrl_code')) {
        this.ctrl_code = initObj.ctrl_code
      }
      else {
        this.ctrl_code = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lli_ctrl_request
    // Serialize message field [steering]
    bufferOffset = _serializer.int8(obj.steering, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.int8(obj.velocity, buffer, bufferOffset);
    // Serialize message field [transmission]
    bufferOffset = _serializer.int8(obj.transmission, buffer, bufferOffset);
    // Serialize message field [differential_front]
    bufferOffset = _serializer.int8(obj.differential_front, buffer, bufferOffset);
    // Serialize message field [differential_rear]
    bufferOffset = _serializer.int8(obj.differential_rear, buffer, bufferOffset);
    // Serialize message field [ctrl_code]
    bufferOffset = _serializer.int8(obj.ctrl_code, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lli_ctrl_request
    let len;
    let data = new lli_ctrl_request(null);
    // Deserialize message field [steering]
    data.steering = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [transmission]
    data.transmission = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [differential_front]
    data.differential_front = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [differential_rear]
    data.differential_rear = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [ctrl_code]
    data.ctrl_code = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'low_level_interface/lli_ctrl_request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '89a86354005db1fc65181bcc6d834320';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 steering
    int8 velocity
    int8 transmission
    int8 differential_front
    int8 differential_rear
    int8 ctrl_code
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lli_ctrl_request(null);
    if (msg.steering !== undefined) {
      resolved.steering = msg.steering;
    }
    else {
      resolved.steering = 0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0
    }

    if (msg.transmission !== undefined) {
      resolved.transmission = msg.transmission;
    }
    else {
      resolved.transmission = 0
    }

    if (msg.differential_front !== undefined) {
      resolved.differential_front = msg.differential_front;
    }
    else {
      resolved.differential_front = 0
    }

    if (msg.differential_rear !== undefined) {
      resolved.differential_rear = msg.differential_rear;
    }
    else {
      resolved.differential_rear = 0
    }

    if (msg.ctrl_code !== undefined) {
      resolved.ctrl_code = msg.ctrl_code;
    }
    else {
      resolved.ctrl_code = 0
    }

    return resolved;
    }
};

module.exports = lli_ctrl_request;
