
(cl:in-package :asdf)

(defsystem "lka-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Lane" :depends-on ("_package_Lane"))
    (:file "_package_Lane" :depends-on ("_package"))
    (:file "Lanes" :depends-on ("_package_Lanes"))
    (:file "_package_Lanes" :depends-on ("_package"))
    (:file "Margins" :depends-on ("_package_Margins"))
    (:file "_package_Margins" :depends-on ("_package"))
  ))