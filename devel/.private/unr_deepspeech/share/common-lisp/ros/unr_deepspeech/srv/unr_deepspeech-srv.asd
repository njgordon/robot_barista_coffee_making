
(cl:in-package :asdf)

(defsystem "unr_deepspeech-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Listen" :depends-on ("_package_Listen"))
    (:file "_package_Listen" :depends-on ("_package"))
  ))