// Auto-generated. Do not edit!

// (in-package pointing_gesture.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class PointingArmJoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.body_id = null;
      this.tracking_status = null;
      this.joint_position_right_elbow = null;
      this.joint_position_right_wrist = null;
    }
    else {
      if (initObj.hasOwnProperty('body_id')) {
        this.body_id = initObj.body_id
      }
      else {
        this.body_id = 0;
      }
      if (initObj.hasOwnProperty('tracking_status')) {
        this.tracking_status = initObj.tracking_status
      }
      else {
        this.tracking_status = 0;
      }
      if (initObj.hasOwnProperty('joint_position_right_elbow')) {
        this.joint_position_right_elbow = initObj.joint_position_right_elbow
      }
      else {
        this.joint_position_right_elbow = new geometry_msgs.msg.Point32();
      }
      if (initObj.hasOwnProperty('joint_position_right_wrist')) {
        this.joint_position_right_wrist = initObj.joint_position_right_wrist
      }
      else {
        this.joint_position_right_wrist = new geometry_msgs.msg.Point32();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PointingArmJoints
    // Serialize message field [body_id]
    bufferOffset = _serializer.int32(obj.body_id, buffer, bufferOffset);
    // Serialize message field [tracking_status]
    bufferOffset = _serializer.int32(obj.tracking_status, buffer, bufferOffset);
    // Serialize message field [joint_position_right_elbow]
    bufferOffset = geometry_msgs.msg.Point32.serialize(obj.joint_position_right_elbow, buffer, bufferOffset);
    // Serialize message field [joint_position_right_wrist]
    bufferOffset = geometry_msgs.msg.Point32.serialize(obj.joint_position_right_wrist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PointingArmJoints
    let len;
    let data = new PointingArmJoints(null);
    // Deserialize message field [body_id]
    data.body_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [tracking_status]
    data.tracking_status = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [joint_position_right_elbow]
    data.joint_position_right_elbow = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_position_right_wrist]
    data.joint_position_right_wrist = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pointing_gesture/PointingArmJoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a9f24fef50af6a7736edb41c88452247';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 body_id           
    int32 tracking_status
    
    geometry_msgs/Point32 joint_position_right_elbow
    geometry_msgs/Point32 joint_position_right_wrist 
    
    
    
    
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PointingArmJoints(null);
    if (msg.body_id !== undefined) {
      resolved.body_id = msg.body_id;
    }
    else {
      resolved.body_id = 0
    }

    if (msg.tracking_status !== undefined) {
      resolved.tracking_status = msg.tracking_status;
    }
    else {
      resolved.tracking_status = 0
    }

    if (msg.joint_position_right_elbow !== undefined) {
      resolved.joint_position_right_elbow = geometry_msgs.msg.Point32.Resolve(msg.joint_position_right_elbow)
    }
    else {
      resolved.joint_position_right_elbow = new geometry_msgs.msg.Point32()
    }

    if (msg.joint_position_right_wrist !== undefined) {
      resolved.joint_position_right_wrist = geometry_msgs.msg.Point32.Resolve(msg.joint_position_right_wrist)
    }
    else {
      resolved.joint_position_right_wrist = new geometry_msgs.msg.Point32()
    }

    return resolved;
    }
};

module.exports = PointingArmJoints;
