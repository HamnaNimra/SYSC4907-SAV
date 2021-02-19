;; Auto-generated. Do not edit!


(when (boundp 'lka::Lanes)
  (if (not (find-package "LKA"))
    (make-package "LKA"))
  (shadow 'Lanes (find-package "LKA")))
(unless (find-package "LKA::LANES")
  (make-package "LKA::LANES"))

(in-package "ROS")
;;//! \htmlinclude Lanes.msg.html


(defclass lka::Lanes
  :super ros::object
  :slots (_lane_lines ))

(defmethod lka::Lanes
  (:init
   (&key
    ((:lane_lines __lane_lines) (let (r) (dotimes (i 2) (push (instance lka::Lane :init) r)) r))
    )
   (send-super :init)
   (setq _lane_lines __lane_lines)
   self)
  (:lane_lines
   (&rest __lane_lines)
   (if (keywordp (car __lane_lines))
       (send* _lane_lines __lane_lines)
     (progn
       (if __lane_lines (setq _lane_lines (car __lane_lines)))
       _lane_lines)))
  (:serialization-length
   ()
   (+
    ;; lka/Lane[2] _lane_lines
    (apply #'+ (send-all _lane_lines :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; lka/Lane[2] _lane_lines
     (dolist (elem _lane_lines)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; lka/Lane[2] _lane_lines
   (dotimes (i 2)
     (send (elt _lane_lines i) :deserialize buf ptr-) (incf ptr- (send (elt _lane_lines i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get lka::Lanes :md5sum-) "d937409978545c3e3e3fe66ef4b4e7c3")
(setf (get lka::Lanes :datatype-) "lka/Lanes")
(setf (get lka::Lanes :definition-)
      "Lane[2] lane_lines
================================================================================
MSG: lka/Lane
bool exists
float64 slope
float64 y_cept

")



(provide :lka/Lanes "d937409978545c3e3e3fe66ef4b4e7c3")


