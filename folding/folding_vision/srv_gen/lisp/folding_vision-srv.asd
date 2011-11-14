
(cl:in-package :asdf)

(defsystem "folding_vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LocatePolygon" :depends-on ("_package_LocatePolygon"))
    (:file "_package_LocatePolygon" :depends-on ("_package"))
  ))