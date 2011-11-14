
(cl:in-package :asdf)

(defsystem "folding_srvs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :folding_msgs-msg
               :geometric_shapes_msgs-msg
               :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "GetTable" :depends-on ("_package_GetTable"))
    (:file "_package_GetTable" :depends-on ("_package"))
    (:file "ExecuteFold" :depends-on ("_package_ExecuteFold"))
    (:file "_package_ExecuteFold" :depends-on ("_package"))
    (:file "GoToGrip" :depends-on ("_package_GoToGrip"))
    (:file "_package_GoToGrip" :depends-on ("_package"))
    (:file "AdjustFold" :depends-on ("_package_AdjustFold"))
    (:file "_package_AdjustFold" :depends-on ("_package"))
    (:file "FoldingStance" :depends-on ("_package_FoldingStance"))
    (:file "_package_FoldingStance" :depends-on ("_package"))
  ))