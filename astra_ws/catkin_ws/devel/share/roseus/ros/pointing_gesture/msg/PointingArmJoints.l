;; Auto-generated. Do not edit!


(when (boundp 'pointing_gesture::PointingArmJoints)
  (if (not (find-package "POINTING_GESTURE"))
    (make-package "POINTING_GESTURE"))
  (shadow 'PointingArmJoints (find-package "POINTING_GESTURE")))
(unless (find-package "POINTING_GESTURE::POINTINGARMJOINTS")
  (make-package "POINTING_GESTURE::POINTINGARMJOINTS"))

(in-package "ROS")
;;//! \htmlinclude PointingArmJoints.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass pointing_gesture::PointingArmJoints
  :super ros::object
  :slots (_body_id _tracking_status _joint_position_right_elbow _joint_position_right_wrist ))

(defmethod pointing_gesture::PointingArmJoints
  (:init
   (&key
    ((:body_id __body_id) 0)
    ((:tracking_status __tracking_status) 0)
    ((:joint_position_right_elbow __joint_position_right_elbow) (instance geometry_msgs::Point32 :init))
    ((:joint_position_right_wrist __joint_position_right_wrist) (instance geometry_msgs::Point32 :init))
    )
   (send-super :init)
   (setq _body_id (round __body_id))
   (setq _tracking_status (round __tracking_status))
   (setq _joint_position_right_elbow __joint_position_right_elbow)
   (setq _joint_position_right_wrist __joint_position_right_wrist)
   self)
  (:body_id
   (&optional __body_id)
   (if __body_id (setq _body_id __body_id)) _body_id)
  (:tracking_status
   (&optional __tracking_status)
   (if __tracking_status (setq _tracking_status __tracking_status)) _tracking_status)
  (:joint_position_right_elbow
   (&rest __joint_position_right_elbow)
   (if (keywordp (car __joint_position_right_elbow))
       (send* _joint_position_right_elbow __joint_position_right_elbow)
     (progn
       (if __joint_position_right_elbow (setq _joint_position_right_elbow (car __joint_position_right_elbow)))
       _joint_position_right_elbow)))
  (:joint_position_right_wrist
   (&rest __joint_position_right_wrist)
   (if (keywordp (car __joint_position_right_wrist))
       (send* _joint_position_right_wrist __joint_position_right_wrist)
     (progn
       (if __joint_position_right_wrist (setq _joint_position_right_wrist (car __joint_position_right_wrist)))
       _joint_position_right_wrist)))
  (:serialization-length
   ()
   (+
    ;; int32 _body_id
    4
    ;; int32 _tracking_status
    4
    ;; geometry_msgs/Point32 _joint_position_right_elbow
    (send _joint_position_right_elbow :serialization-length)
    ;; geometry_msgs/Point32 _joint_position_right_wrist
    (send _joint_position_right_wrist :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _body_id
       (write-long _body_id s)
     ;; int32 _tracking_status
       (write-long _tracking_status s)
     ;; geometry_msgs/Point32 _joint_position_right_elbow
       (send _joint_position_right_elbow :serialize s)
     ;; geometry_msgs/Point32 _joint_position_right_wrist
       (send _joint_position_right_wrist :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _body_id
     (setq _body_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _tracking_status
     (setq _tracking_status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point32 _joint_position_right_elbow
     (send _joint_position_right_elbow :deserialize buf ptr-) (incf ptr- (send _joint_position_right_elbow :serialization-length))
   ;; geometry_msgs/Point32 _joint_position_right_wrist
     (send _joint_position_right_wrist :deserialize buf ptr-) (incf ptr- (send _joint_position_right_wrist :serialization-length))
   ;;
   self)
  )

(setf (get pointing_gesture::PointingArmJoints :md5sum-) "a9f24fef50af6a7736edb41c88452247")
(setf (get pointing_gesture::PointingArmJoints :datatype-) "pointing_gesture/PointingArmJoints")
(setf (get pointing_gesture::PointingArmJoints :definition-)
      "int32 body_id           
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
")



(provide :pointing_gesture/PointingArmJoints "a9f24fef50af6a7736edb41c88452247")


