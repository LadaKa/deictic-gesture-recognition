// Auto-generated. Do not edit!

// (in-package pcl_object_detection.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class DetectedObjects {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.objectsCenters = null;
    }
    else {
      if (initObj.hasOwnProperty('objectsCenters')) {
        this.objectsCenters = initObj.objectsCenters
      }
      else {
        this.objectsCenters = new Array(3).fill(new geometry_msgs.msg.Point32());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectedObjects
    // Check that the constant length array field [objectsCenters] has the right length
    if (obj.objectsCenters.length !== 3) {
      throw new Error('Unable to serialize array field objectsCenters - length must be 3')
    }
    // Serialize message field [objectsCenters]
    obj.objectsCenters.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point32.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectedObjects
    let len;
    let data = new DetectedObjects(null);
    // Deserialize message field [objectsCenters]
    len = 3;
    data.objectsCenters = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.objectsCenters[i] = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pcl_object_detection/DetectedObjects';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '17094eefbe3742486c3f60ffb5186309';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point32[3] objectsCenters
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
    const resolved = new DetectedObjects(null);
    if (msg.objectsCenters !== undefined) {
      resolved.objectsCenters = new Array(3)
      for (let i = 0; i < resolved.objectsCenters.length; ++i) {
        if (msg.objectsCenters.length > i) {
          resolved.objectsCenters[i] = geometry_msgs.msg.Point32.Resolve(msg.objectsCenters[i]);
        }
        else {
          resolved.objectsCenters[i] = new geometry_msgs.msg.Point32();
        }
      }
    }
    else {
      resolved.objectsCenters = new Array(3).fill(new geometry_msgs.msg.Point32())
    }

    return resolved;
    }
};

module.exports = DetectedObjects;
