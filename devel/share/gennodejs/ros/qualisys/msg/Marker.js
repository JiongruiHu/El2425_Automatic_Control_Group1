// Auto-generated. Do not edit!

// (in-package qualisys.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Marker {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.subject_name = null;
      this.position = null;
      this.occluded = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('subject_name')) {
        this.subject_name = initObj.subject_name
      }
      else {
        this.subject_name = '';
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('occluded')) {
        this.occluded = initObj.occluded
      }
      else {
        this.occluded = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Marker
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [subject_name]
    bufferOffset = _serializer.string(obj.subject_name, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [occluded]
    bufferOffset = _serializer.bool(obj.occluded, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Marker
    let len;
    let data = new Marker(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [subject_name]
    data.subject_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [occluded]
    data.occluded = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    length += object.subject_name.length;
    return length + 33;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qualisys/Marker';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a16c57ea269b234761b832931693cc90';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    string subject_name
    geometry_msgs/Point position
    bool occluded
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Marker(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.subject_name !== undefined) {
      resolved.subject_name = msg.subject_name;
    }
    else {
      resolved.subject_name = ''
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.occluded !== undefined) {
      resolved.occluded = msg.occluded;
    }
    else {
      resolved.occluded = false
    }

    return resolved;
    }
};

module.exports = Marker;
