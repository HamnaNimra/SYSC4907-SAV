;; Auto-generated. Do not edit!


(when (boundp 'controls::Control)
  (if (not (find-package "CONTROLS"))
    (make-package "CONTROLS"))
  (shadow 'Control (find-package "CONTROLS")))
(unless (find-package "CONTROLS::CONTROL")
  (make-package "CONTROLS::CONTROL"))

(in-package "ROS")
;;//! \htmlinclude Control.msg.html


(defclass controls::Control
  :super ros::object
  :slots (_brake _throttle _steering ))

(defmethod controls::Control
  (:init
   (&key
    ((:brake __brake) 0.0)
    ((:throttle __throttle) 0.0)
    ((:steering __steering) 0.0)
    )
   (send-super :init)
   (setq _brake (float __brake))
   (setq _throttle (float __throttle))
   (setq _steering (float __steering))
   self)
  (:brake
   (&optional __brake)
   (if __brake (setq _brake __brake)) _brake)
  (:throttle
   (&optional __throttle)
   (if __throttle (setq _throttle __throttle)) _throttle)
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:serialization-length
   ()
   (+
    ;; float64 _brake
    8
    ;; float64 _throttle
    8
    ;; float64 _steering
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _brake
       (sys::poke _brake (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _throttle
       (sys::poke _throttle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steering
       (sys::poke _steering (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _brake
     (setq _brake (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _throttle
     (setq _throttle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steering
     (setq _steering (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get controls::Control :md5sum-) "a50e54d6f035ba47c042e73d1ecdc700")
(setf (get controls::Control :datatype-) "controls/Control")
(setf (get controls::Control :definition-)
      "float64 brake
float64 throttle
float64 steering
")



(provide :controls/Control "a50e54d6f035ba47c042e73d1ecdc700")


