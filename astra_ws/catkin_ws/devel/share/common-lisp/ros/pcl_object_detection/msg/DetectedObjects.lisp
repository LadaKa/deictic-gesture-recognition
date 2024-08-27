; Auto-generated. Do not edit!


(cl:in-package pcl_object_detection-msg)


;//! \htmlinclude DetectedObjects.msg.html

(cl:defclass <DetectedObjects> (roslisp-msg-protocol:ros-message)
  ((objectsCenters
    :reader objectsCenters
    :initarg :objectsCenters
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 3 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32))))
)

(cl:defclass DetectedObjects (<DetectedObjects>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectedObjects>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectedObjects)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pcl_object_detection-msg:<DetectedObjects> is deprecated: use pcl_object_detection-msg:DetectedObjects instead.")))

(cl:ensure-generic-function 'objectsCenters-val :lambda-list '(m))
(cl:defmethod objectsCenters-val ((m <DetectedObjects>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pcl_object_detection-msg:objectsCenters-val is deprecated.  Use pcl_object_detection-msg:objectsCenters instead.")
  (objectsCenters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectedObjects>) ostream)
  "Serializes a message object of type '<DetectedObjects>"
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objectsCenters))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectedObjects>) istream)
  "Deserializes a message object of type '<DetectedObjects>"
  (cl:setf (cl:slot-value msg 'objectsCenters) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'objectsCenters)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectedObjects>)))
  "Returns string type for a message object of type '<DetectedObjects>"
  "pcl_object_detection/DetectedObjects")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectedObjects)))
  "Returns string type for a message object of type 'DetectedObjects"
  "pcl_object_detection/DetectedObjects")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectedObjects>)))
  "Returns md5sum for a message object of type '<DetectedObjects>"
  "17094eefbe3742486c3f60ffb5186309")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectedObjects)))
  "Returns md5sum for a message object of type 'DetectedObjects"
  "17094eefbe3742486c3f60ffb5186309")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectedObjects>)))
  "Returns full string definition for message of type '<DetectedObjects>"
  (cl:format cl:nil "geometry_msgs/Point32[3] objectsCenters~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectedObjects)))
  "Returns full string definition for message of type 'DetectedObjects"
  (cl:format cl:nil "geometry_msgs/Point32[3] objectsCenters~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectedObjects>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'objectsCenters) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectedObjects>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectedObjects
    (cl:cons ':objectsCenters (objectsCenters msg))
))
