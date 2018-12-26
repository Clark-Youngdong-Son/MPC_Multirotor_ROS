
(cl:in-package :asdf)

(defsystem "mpc_icra-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Attitude" :depends-on ("_package_Attitude"))
    (:file "_package_Attitude" :depends-on ("_package"))
    (:file "SlungLoadState" :depends-on ("_package_SlungLoadState"))
    (:file "_package_SlungLoadState" :depends-on ("_package"))
    (:file "Position" :depends-on ("_package_Position"))
    (:file "_package_Position" :depends-on ("_package"))
    (:file "Gain" :depends-on ("_package_Gain"))
    (:file "_package_Gain" :depends-on ("_package"))
  ))