; Auto-generated. Do not edit!


(cl:in-package line_follower_turtlebot-msg)


;//! \htmlinclude pos.msg.html

(cl:defclass <pos> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:float
    :initform 0.0))
)

(cl:defclass pos (<pos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name line_follower_turtlebot-msg:<pos> is deprecated: use line_follower_turtlebot-msg:pos instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader line_follower_turtlebot-msg:direction-val is deprecated.  Use line_follower_turtlebot-msg:direction instead.")
  (direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos>) ostream)
  "Serializes a message object of type '<pos>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos>) istream)
  "Deserializes a message object of type '<pos>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'direction) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos>)))
  "Returns string type for a message object of type '<pos>"
  "line_follower_turtlebot/pos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos)))
  "Returns string type for a message object of type 'pos"
  "line_follower_turtlebot/pos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos>)))
  "Returns md5sum for a message object of type '<pos>"
  "027b9297308a5b7b690348b04b1998f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos)))
  "Returns md5sum for a message object of type 'pos"
  "027b9297308a5b7b690348b04b1998f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos>)))
  "Returns full string definition for message of type '<pos>"
  (cl:format cl:nil "float64 direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos)))
  "Returns full string definition for message of type 'pos"
  (cl:format cl:nil "float64 direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos>))
  "Converts a ROS message object to a list"
  (cl:list 'pos
    (cl:cons ':direction (direction msg))
))
