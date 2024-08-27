; Auto-generated. Do not edit!


(cl:in-package pointing_gesture-msg)


;//! \htmlinclude PointingArmJoints.msg.html

(cl:defclass <PointingArmJoints> (roslisp-msg-protocol:ros-message)
  ((body_id
    :reader body_id
    :initarg :body_id
    :type cl:integer
    :initform 0)
   (tracking_status
    :reader tracking_status
    :initarg :tracking_status
    :type cl:integer
    :initform 0)
   (joint_position_right_elbow
    :reader joint_position_right_elbow
    :initarg :joint_position_right_elbow
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (joint_position_right_wrist
    :reader joint_position_right_wrist
    :initarg :joint_position_right_wrist
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass PointingArmJoints (<PointingArmJoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointingArmJoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointingArmJoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pointing_gesture-msg:<PointingArmJoints> is deprecated: use pointing_gesture-msg:PointingArmJoints instead.")))

(cl:ensure-generic-function 'body_id-val :lambda-list '(m))
(cl:defmethod body_id-val ((m <PointingArmJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pointing_gesture-msg:body_id-val is deprecated.  Use pointing_gesture-msg:body_id instead.")
  (body_id m))

(cl:ensure-generic-function 'tracking_status-val :lambda-list '(m))
(cl:defmethod tracking_status-val ((m <PointingArmJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pointing_gesture-msg:tracking_status-val is deprecated.  Use pointing_gesture-msg:tracking_status instead.")
  (tracking_status m))

(cl:ensure-generic-function 'joint_position_right_elbow-val :lambda-list '(m))
(cl:defmethod joint_position_right_elbow-val ((m <PointingArmJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pointing_gesture-msg:joint_position_right_elbow-val is deprecated.  Use pointing_gesture-msg:joint_position_right_elbow instead.")
  (joint_position_right_elbow m))

(cl:ensure-generic-function 'joint_position_right_wrist-val :lambda-list '(m))
(cl:defmethod joint_position_right_wrist-val ((m <PointingArmJoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pointing_gesture-msg:joint_position_right_wrist-val is deprecated.  Use pointing_gesture-msg:joint_position_right_wrist instead.")
  (joint_position_right_wrist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointingArmJoints>) ostream)
  "Serializes a message object of type '<PointingArmJoints>"
  (cl:let* ((signed (cl:slot-value msg 'body_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tracking_status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_position_right_elbow) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_position_right_wrist) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointingArmJoints>) istream)
  "Deserializes a message object of type '<PointingArmJoints>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'body_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tracking_status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_position_right_elbow) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_position_right_wrist) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointingArmJoints>)))
  "Returns string type for a message object of type '<PointingArmJoints>"
  "pointing_gesture/PointingArmJoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointingArmJoints)))
  "Returns string type for a message object of type 'PointingArmJoints"
  "pointing_gesture/PointingArmJoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointingArmJoints>)))
  "Returns md5sum for a message object of type '<PointingArmJoints>"
  "a9f24fef50af6a7736edb41c88452247")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointingArmJoints)))
  "Returns md5sum for a message object of type 'PointingArmJoints"
  "a9f24fef50af6a7736edb41c88452247")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointingArmJoints>)))
  "Returns full string definition for message of type '<PointingArmJoints>"
  (cl:format cl:nil "int32 body_id           ~%int32 tracking_status~%~%geometry_msgs/Point32 joint_position_right_elbow~%geometry_msgs/Point32 joint_position_right_wrist ~%~%~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointingArmJoints)))
  "Returns full string definition for message of type 'PointingArmJoints"
  (cl:format cl:nil "int32 body_id           ~%int32 tracking_status~%~%geometry_msgs/Point32 joint_position_right_elbow~%geometry_msgs/Point32 joint_position_right_wrist ~%~%~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointingArmJoints>))
  (cl:+ 0
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_position_right_elbow))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_position_right_wrist))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointingArmJoints>))
  "Converts a ROS message object to a list"
  (cl:list 'PointingArmJoints
    (cl:cons ':body_id (body_id msg))
    (cl:cons ':tracking_status (tracking_status msg))
    (cl:cons ':joint_position_right_elbow (joint_position_right_elbow msg))
    (cl:cons ':joint_position_right_wrist (joint_position_right_wrist msg))
))
