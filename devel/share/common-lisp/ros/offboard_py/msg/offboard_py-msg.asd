
(cl:in-package :asdf)

(defsystem "offboard_py-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "position3D" :depends-on ("_package_position3D"))
    (:file "_package_position3D" :depends-on ("_package"))
  ))