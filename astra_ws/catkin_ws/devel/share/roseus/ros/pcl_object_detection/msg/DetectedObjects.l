;; Auto-generated. Do not edit!


(when (boundp 'pcl_object_detection::DetectedObjects)
  (if (not (find-package "PCL_OBJECT_DETECTION"))
    (make-package "PCL_OBJECT_DETECTION"))
  (shadow 'DetectedObjects (find-package "PCL_OBJECT_DETECTION")))
(unless (find-package "PCL_OBJECT_DETECTION::DETECTEDOBJECTS")
  (make-package "PCL_OBJECT_DETECTION::DETECTEDOBJECTS"))

(in-package "ROS")
;;//! \htmlinclude DetectedObjects.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass pcl_object_detection::DetectedObjects
  :super ros::object
  :slots (_objectsCenters ))

(defmethod pcl_object_detection::DetectedObjects
  (:init
   (&key
    ((:objectsCenters __objectsCenters) (let (r) (dotimes (i 3) (push (instance geometry_msgs::Point32 :init) r)) r))
    )
   (send-super :init)
   (setq _objectsCenters __objectsCenters)
   self)
  (:objectsCenters
   (&rest __objectsCenters)
   (if (keywordp (car __objectsCenters))
       (send* _objectsCenters __objectsCenters)
     (progn
       (if __objectsCenters (setq _objectsCenters (car __objectsCenters)))
       _objectsCenters)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point32[3] _objectsCenters
    (apply #'+ (send-all _objectsCenters :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point32[3] _objectsCenters
     (dolist (elem _objectsCenters)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point32[3] _objectsCenters
   (dotimes (i 3)
     (send (elt _objectsCenters i) :deserialize buf ptr-) (incf ptr- (send (elt _objectsCenters i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get pcl_object_detection::DetectedObjects :md5sum-) "17094eefbe3742486c3f60ffb5186309")
(setf (get pcl_object_detection::DetectedObjects :datatype-) "pcl_object_detection/DetectedObjects")
(setf (get pcl_object_detection::DetectedObjects :definition-)
      "geometry_msgs/Point32[3] objectsCenters
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



(provide :pcl_object_detection/DetectedObjects "17094eefbe3742486c3f60ffb5186309")


