;; Auto-generated. Do not edit!


(when (boundp 'lka::Lane)
  (if (not (find-package "LKA"))
    (make-package "LKA"))
  (shadow 'Lane (find-package "LKA")))
(unless (find-package "LKA::LANE")
  (make-package "LKA::LANE"))

(in-package "ROS")
;;//! \htmlinclude Lane.msg.html


(defclass lka::Lane
  :super ros::object
  :slots (_exists _slope _y_cept ))

(defmethod lka::Lane
  (:init
   (&key
    ((:exists __exists) nil)
    ((:slope __slope) 0.0)
    ((:y_cept __y_cept) 0.0)
    )
   (send-super :init)
   (setq _exists __exists)
   (setq _slope (float __slope))
   (setq _y_cept (float __y_cept))
   self)
  (:exists
   (&optional __exists)
   (if __exists (setq _exists __exists)) _exists)
  (:slope
   (&optional __slope)
   (if __slope (setq _slope __slope)) _slope)
  (:y_cept
   (&optional __y_cept)
   (if __y_cept (setq _y_cept __y_cept)) _y_cept)
  (:serialization-length
   ()
   (+
    ;; bool _exists
    1
    ;; float64 _slope
    8
    ;; float64 _y_cept
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _exists
       (if _exists (write-byte -1 s) (write-byte 0 s))
     ;; float64 _slope
       (sys::poke _slope (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_cept
       (sys::poke _y_cept (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _exists
     (setq _exists (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _slope
     (setq _slope (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_cept
     (setq _y_cept (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get lka::Lane :md5sum-) "d8fffbc133615afc331c7af31fc633d4")
(setf (get lka::Lane :datatype-) "lka/Lane")
(setf (get lka::Lane :definition-)
      "bool exists
float64 slope
float64 y_cept

")



(provide :lka/Lane "d8fffbc133615afc331c7af31fc633d4")


