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

class lli_ctrl_actuated {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.steering = null;
      this.velocity = null;
      this.transmission = null;
      this.differential_front = null;
      this.differential_rear = null;
      this.ctrl_code = null;
      this.voltage = null;
      this.current = null;
      this.energy_consumed = null;
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
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0;
      }
      if (initObj.hasOwnProperty('energy_consumed')) {
        this.energy_consumed = initObj.energy_consumed
      }
      else {
        this.energy_consumed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lli_ctrl_actuated
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
    // Serialize message field [voltage]
    bufferOffset = _serializer.int8(obj.voltage, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.int8(obj.current, buffer, bufferOffset);
    // Serialize message field [energy_consumed]
    bufferOffset = _serializer.int8(obj.energy_consumed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lli_ctrl_actuated
    let len;
    let data = new lli_ctrl_actuated(null);
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
    // Deserialize message field [voltage]
    data.voltage = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [energy_consumed]
    data.energy_consumed = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'low_level_interface/lli_ctrl_actuated';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6c801b9e27d9831fee1704e6d7985df4';
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
    int8 voltage
    int8 current
    int8 energy_consumed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lli_ctrl_actuated(null);
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

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0
    }

    if (msg.energy_consumed !== undefined) {
      resolved.energy_consumed = msg.energy_consumed;
    }
    else {
      resolved.energy_consumed = 0
    }

    return resolved;
    }
};

module.exports = lli_ctrl_actuated;
