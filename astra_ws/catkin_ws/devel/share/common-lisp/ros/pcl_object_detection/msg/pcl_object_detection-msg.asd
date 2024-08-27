
(cl:in-package :asdf)

(defsystem "pcl_object_detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "DetectedObjects" :depends-on ("_package_DetectedObjects"))
    (:file "_package_DetectedObjects" :depends-on ("_package"))
  ))