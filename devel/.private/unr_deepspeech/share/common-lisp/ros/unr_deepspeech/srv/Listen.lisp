; Auto-generated. Do not edit!


(cl:in-package unr_deepspeech-srv)


;//! \htmlinclude Listen-request.msg.html

(cl:defclass <Listen-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass Listen-request (<Listen-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Listen-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Listen-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unr_deepspeech-srv:<Listen-request> is deprecated: use unr_deepspeech-srv:Listen-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <Listen-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unr_deepspeech-srv:filename-val is deprecated.  Use unr_deepspeech-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Listen-request>) ostream)
  "Serializes a message object of type '<Listen-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Listen-request>) istream)
  "Deserializes a message object of type '<Listen-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Listen-request>)))
  "Returns string type for a service object of type '<Listen-request>"
  "unr_deepspeech/ListenRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Listen-request)))
  "Returns string type for a service object of type 'Listen-request"
  "unr_deepspeech/ListenRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Listen-request>)))
  "Returns md5sum for a message object of type '<Listen-request>"
  "d3b16d8cd85e65cb9eefd2c2099a8d05")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Listen-request)))
  "Returns md5sum for a message object of type 'Listen-request"
  "d3b16d8cd85e65cb9eefd2c2099a8d05")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Listen-request>)))
  "Returns full string definition for message of type '<Listen-request>"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Listen-request)))
  "Returns full string definition for message of type 'Listen-request"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Listen-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Listen-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Listen-request
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude Listen-response.msg.html

(cl:defclass <Listen-response> (roslisp-msg-protocol:ros-message)
  ((prediction
    :reader prediction
    :initarg :prediction
    :type cl:string
    :initform ""))
)

(cl:defclass Listen-response (<Listen-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Listen-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Listen-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unr_deepspeech-srv:<Listen-response> is deprecated: use unr_deepspeech-srv:Listen-response instead.")))

(cl:ensure-generic-function 'prediction-val :lambda-list '(m))
(cl:defmethod prediction-val ((m <Listen-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unr_deepspeech-srv:prediction-val is deprecated.  Use unr_deepspeech-srv:prediction instead.")
  (prediction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Listen-response>) ostream)
  "Serializes a message object of type '<Listen-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'prediction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'prediction))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Listen-response>) istream)
  "Deserializes a message object of type '<Listen-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'prediction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'prediction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Listen-response>)))
  "Returns string type for a service object of type '<Listen-response>"
  "unr_deepspeech/ListenResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Listen-response)))
  "Returns string type for a service object of type 'Listen-response"
  "unr_deepspeech/ListenResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Listen-response>)))
  "Returns md5sum for a message object of type '<Listen-response>"
  "d3b16d8cd85e65cb9eefd2c2099a8d05")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Listen-response)))
  "Returns md5sum for a message object of type 'Listen-response"
  "d3b16d8cd85e65cb9eefd2c2099a8d05")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Listen-response>)))
  "Returns full string definition for message of type '<Listen-response>"
  (cl:format cl:nil "string prediction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Listen-response)))
  "Returns full string definition for message of type 'Listen-response"
  (cl:format cl:nil "string prediction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Listen-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'prediction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Listen-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Listen-response
    (cl:cons ':prediction (prediction msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Listen)))
  'Listen-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Listen)))
  'Listen-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Listen)))
  "Returns string type for a service object of type '<Listen>"
  "unr_deepspeech/Listen")