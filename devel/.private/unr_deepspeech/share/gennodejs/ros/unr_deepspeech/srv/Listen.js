// Auto-generated. Do not edit!

// (in-package unr_deepspeech.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ListenRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.filename = null;
    }
    else {
      if (initObj.hasOwnProperty('filename')) {
        this.filename = initObj.filename
      }
      else {
        this.filename = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ListenRequest
    // Serialize message field [filename]
    bufferOffset = _serializer.string(obj.filename, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ListenRequest
    let len;
    let data = new ListenRequest(null);
    // Deserialize message field [filename]
    data.filename = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.filename.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'unr_deepspeech/ListenRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '030824f52a0628ead956fb9d67e66ae9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string filename
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ListenRequest(null);
    if (msg.filename !== undefined) {
      resolved.filename = msg.filename;
    }
    else {
      resolved.filename = ''
    }

    return resolved;
    }
};

class ListenResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.prediction = null;
    }
    else {
      if (initObj.hasOwnProperty('prediction')) {
        this.prediction = initObj.prediction
      }
      else {
        this.prediction = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ListenResponse
    // Serialize message field [prediction]
    bufferOffset = _serializer.string(obj.prediction, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ListenResponse
    let len;
    let data = new ListenResponse(null);
    // Deserialize message field [prediction]
    data.prediction = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.prediction.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'unr_deepspeech/ListenResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44a298085da4e117b1416bb683e65e00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string prediction
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ListenResponse(null);
    if (msg.prediction !== undefined) {
      resolved.prediction = msg.prediction;
    }
    else {
      resolved.prediction = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ListenRequest,
  Response: ListenResponse,
  md5sum() { return 'd3b16d8cd85e65cb9eefd2c2099a8d05'; },
  datatype() { return 'unr_deepspeech/Listen'; }
};
