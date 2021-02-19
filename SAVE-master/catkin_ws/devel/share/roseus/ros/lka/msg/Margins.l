;; Auto-generated. Do not edit!


(when (boundp 'lka::Margins)
  (if (not (find-package "LKA"))
    (make-package "LKA"))
  (shadow 'Margins (find-package "LKA")))
(unless (find-package "LKA::MARGINS")
  (make-package "LKA::MARGINS"))

(in-package "ROS")
;;//! \htmlinclude Margins.msg.html


(defclass lka::Margins
  :super ros::object
  :slots (_margin_diff ))

(defmethod lka::Margins
  (:init
   (&key
    ((:margin_diff __margin_diff) 0.0)
    )
   (send-super :init)
   (setq _margin_diff (float __margin_diff))
   self)
  (:margin_diff
   (&optional __margin_diff)
   (if __margin_diff (setq _margin_diff __margin_diff)) _margin_diff)
  (:serialization-length
   ()
   (+
    ;; float64 _margin_diff
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _margin_diff
       (sys::poke _margin_diff (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _margin_diff
     (setq _margin_diff (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get lka::Margins :md5sum-) "8a2be87eb93522a3eb44d383e70dffc2")
(setf (get lka::Margins :datatype-) "lka/Margins")
(setf (get lka::Margins :definition-)
      "float64 margin_diff
")



(provide :lka/Margins "8a2be87eb93522a3eb44d383e70dffc2")


