
(cl:in-package :asdf)

(defsystem "folding_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Line2D" :depends-on ("_package_Line2D"))
    (:file "_package_Line2D" :depends-on ("_package"))
    (:file "PolyStamped" :depends-on ("_package_PolyStamped"))
    (:file "_package_PolyStamped" :depends-on ("_package"))
    (:file "GripTarget" :depends-on ("_package_GripTarget"))
    (:file "_package_GripTarget" :depends-on ("_package"))
    (:file "FoldTraj" :depends-on ("_package_FoldTraj"))
    (:file "_package_FoldTraj" :depends-on ("_package"))
    (:file "Point2D" :depends-on ("_package_Point2D"))
    (:file "_package_Point2D" :depends-on ("_package"))
    (:file "GripsTarget" :depends-on ("_package_GripsTarget"))
    (:file "_package_GripsTarget" :depends-on ("_package"))
  ))