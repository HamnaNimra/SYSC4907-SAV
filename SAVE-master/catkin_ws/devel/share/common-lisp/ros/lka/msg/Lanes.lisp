; Auto-generated. Do not edit!


(cl:in-package lka-msg)


;//! \htmlinclude Lanes.msg.html

(cl:defclass <Lanes> (roslisp-msg-protocol:ros-message)
  ((lane_lines
    :reader lane_lines
    :initarg :lane_lines
    :type (cl:vector lka-msg:Lane)
   :initform (cl:make-array 2 :element-type 'lka-msg:Lane :initial-element (cl:make-instance 'lka-msg:Lane))))
)

(cl:defclass Lanes (<Lanes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Lanes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Lanes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lka-msg:<Lanes> is deprecated: use lka-msg:Lanes instead.")))

(cl:ensure-generic-function 'lane_lines-val :lambda-list '(m))
(cl:defmethod lane_lines-val ((m <Lanes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lka-msg:lane_lines-val is deprecated.  Use lka-msg:lane_lines instead.")
  (lane_lines m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Lanes>) ostream)
  "Serializes a message object of type '<Lanes>"
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'lane_lines))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Lanes>) istream)
  "Deserializes a message object of type '<Lanes>"
  (cl:setf (cl:slot-value msg 'lane_lines) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'lane_lines)))
    (cl:dotimes (i 2)
    (cl:setf (cl:aref vals i) (cl:make-instance 'lka-msg:Lane))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Lanes>)))
  "Returns string type for a message object of type '<Lanes>"
  "lka/Lanes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Lanes)))
  "Returns string type for a message object of type 'Lanes"
  "lka/Lanes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Lanes>)))
  "Returns md5sum for a message object of type '<Lanes>"
  "d937409978545c3e3e3fe66ef4b4e7c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Lanes)))
  "Returns md5sum for a message object of type 'Lanes"
  "d937409978545c3e3e3fe66ef4b4e7c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Lanes>)))
  "Returns full string definition for message of type '<Lanes>"
  (cl:format cl:nil "Lane[2] lane_lines~%================================================================================~%MSG: lka/Lane~%bool exists~%float64 slope~%float64 y_cept~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Lanes)))
  "Returns full string definition for message of type 'Lanes"
  (cl:format cl:nil "Lane[2] lane_lines~%================================================================================~%MSG: lka/Lane~%bool exists~%float64 slope~%float64 y_cept~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Lanes>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lane_lines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Lanes>))
  "Converts a ROS message object to a list"
  (cl:list 'Lanes
    (cl:cons ':lane_lines (lane_lines msg))
))
