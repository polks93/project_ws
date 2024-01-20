// Auto-generated. Do not edit!

// (in-package my_package.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class WaypointDistanceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_id = null;
      this.waypoint_id = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_id')) {
        this.robot_id = initObj.robot_id
      }
      else {
        this.robot_id = 0;
      }
      if (initObj.hasOwnProperty('waypoint_id')) {
        this.waypoint_id = initObj.waypoint_id
      }
      else {
        this.waypoint_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WaypointDistanceRequest
    // Serialize message field [robot_id]
    bufferOffset = _serializer.uint8(obj.robot_id, buffer, bufferOffset);
    // Serialize message field [waypoint_id]
    bufferOffset = _serializer.uint8(obj.waypoint_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WaypointDistanceRequest
    let len;
    let data = new WaypointDistanceRequest(null);
    // Deserialize message field [robot_id]
    data.robot_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [waypoint_id]
    data.waypoint_id = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_package/WaypointDistanceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '831c18e1bdee664e0ac4ccc25433d3c2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8    robot_id
    uint8    waypoint_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WaypointDistanceRequest(null);
    if (msg.robot_id !== undefined) {
      resolved.robot_id = msg.robot_id;
    }
    else {
      resolved.robot_id = 0
    }

    if (msg.waypoint_id !== undefined) {
      resolved.waypoint_id = msg.waypoint_id;
    }
    else {
      resolved.waypoint_id = 0
    }

    return resolved;
    }
};

module.exports = WaypointDistanceRequest;
