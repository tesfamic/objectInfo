
(cl:in-package :asdf)

(defsystem "objectDetector0-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "objectPos" :depends-on ("_package_objectPos"))
    (:file "_package_objectPos" :depends-on ("_package"))
    (:file "objectPosArray" :depends-on ("_package_objectPosArray"))
    (:file "_package_objectPosArray" :depends-on ("_package"))
  ))