;; Auto-generated. Do not edit!


(when (boundp 'unr_deepspeech::Listen)
  (if (not (find-package "UNR_DEEPSPEECH"))
    (make-package "UNR_DEEPSPEECH"))
  (shadow 'Listen (find-package "UNR_DEEPSPEECH")))
(unless (find-package "UNR_DEEPSPEECH::LISTEN")
  (make-package "UNR_DEEPSPEECH::LISTEN"))
(unless (find-package "UNR_DEEPSPEECH::LISTENREQUEST")
  (make-package "UNR_DEEPSPEECH::LISTENREQUEST"))
(unless (find-package "UNR_DEEPSPEECH::LISTENRESPONSE")
  (make-package "UNR_DEEPSPEECH::LISTENRESPONSE"))

(in-package "ROS")





(defclass unr_deepspeech::ListenRequest
  :super ros::object
  :slots (_filename ))

(defmethod unr_deepspeech::ListenRequest
  (:init
   (&key
    ((:filename __filename) "")
    )
   (send-super :init)
   (setq _filename (string __filename))
   self)
  (:filename
   (&optional __filename)
   (if __filename (setq _filename __filename)) _filename)
  (:serialization-length
   ()
   (+
    ;; string _filename
    4 (length _filename)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _filename
       (write-long (length _filename) s) (princ _filename s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _filename
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _filename (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass unr_deepspeech::ListenResponse
  :super ros::object
  :slots (_prediction ))

(defmethod unr_deepspeech::ListenResponse
  (:init
   (&key
    ((:prediction __prediction) "")
    )
   (send-super :init)
   (setq _prediction (string __prediction))
   self)
  (:prediction
   (&optional __prediction)
   (if __prediction (setq _prediction __prediction)) _prediction)
  (:serialization-length
   ()
   (+
    ;; string _prediction
    4 (length _prediction)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _prediction
       (write-long (length _prediction) s) (princ _prediction s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _prediction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _prediction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass unr_deepspeech::Listen
  :super ros::object
  :slots ())

(setf (get unr_deepspeech::Listen :md5sum-) "d3b16d8cd85e65cb9eefd2c2099a8d05")
(setf (get unr_deepspeech::Listen :datatype-) "unr_deepspeech/Listen")
(setf (get unr_deepspeech::Listen :request) unr_deepspeech::ListenRequest)
(setf (get unr_deepspeech::Listen :response) unr_deepspeech::ListenResponse)

(defmethod unr_deepspeech::ListenRequest
  (:response () (instance unr_deepspeech::ListenResponse :init)))

(setf (get unr_deepspeech::ListenRequest :md5sum-) "d3b16d8cd85e65cb9eefd2c2099a8d05")
(setf (get unr_deepspeech::ListenRequest :datatype-) "unr_deepspeech/ListenRequest")
(setf (get unr_deepspeech::ListenRequest :definition-)
      "string filename
---
string prediction
")

(setf (get unr_deepspeech::ListenResponse :md5sum-) "d3b16d8cd85e65cb9eefd2c2099a8d05")
(setf (get unr_deepspeech::ListenResponse :datatype-) "unr_deepspeech/ListenResponse")
(setf (get unr_deepspeech::ListenResponse :definition-)
      "string filename
---
string prediction
")



(provide :unr_deepspeech/Listen "d3b16d8cd85e65cb9eefd2c2099a8d05")


