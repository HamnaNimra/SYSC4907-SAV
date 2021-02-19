;; Auto-generated. Do not edit!


(when (boundp 'sensors::Speed)
  (if (not (find-package "SENSORS"))
    (make-package "SENSORS"))
  (shadow 'Speed (find-package "SENSORS")))
(unless (find-package "SENSORS::SPEED")
  (make-package "SENSORS::SPEED"))

(in-package "ROS")
;;//! \htmlinclude Speed.msg.html


(defclass sensors::Speed
  :super ros::object
  :slots (_speed ))

(defmethod sensors::Speed
  (:init
   (&key
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _speed (float __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float64 _speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get sensors::Speed :md5sum-) "4641bb0523a3557209606d9bd91ce33a")
(setf (get sensors::Speed :datatype-) "sensors/Speed")
(setf (get sensors::Speed :definition-)
      "float64 speed
")



(provide :sensors/Speed "4641bb0523a3557209606d9bd91ce33a")


