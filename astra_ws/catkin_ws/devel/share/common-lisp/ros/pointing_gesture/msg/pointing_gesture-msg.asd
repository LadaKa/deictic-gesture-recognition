
(cl:in-package :asdf)

(defsystem "pointing_gesture-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PointingArmJoints" :depends-on ("_package_PointingArmJoints"))
    (:file "_package_PointingArmJoints" :depends-on ("_package"))
  ))