; Auto-generated. Do not edit!


(cl:in-package folding_msgs-msg)


;//! \htmlinclude GripsTarget.msg.html

(cl:defclass <GripsTarget> (roslisp-msg-protocol:ros-message)
  ((l_target
    :reader l_target
    :initarg :l_target
    :type folding_msgs-msg:GripTarget
    :initform (cl:make-instance 'folding_msgs-msg:GripTarget))
   (r_target
    :reader r_target
    :initarg :r_target
    :type folding_msgs-msg:GripTarget
    :initform (cl:make-instance 'folding_msgs-msg:GripTarget)))
)

(cl:defclass GripsTarget (<GripsTarget>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripsTarget>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripsTarget)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_msgs-msg:<GripsTarget> is deprecated: use folding_msgs-msg:GripsTarget instead.")))

(cl:ensure-generic-function 'l_target-val :lambda-list '(m))
(cl:defmethod l_target-val ((m <GripsTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:l_target-val is deprecated.  Use folding_msgs-msg:l_target instead.")
  (l_target m))

(cl:ensure-generic-function 'r_target-val :lambda-list '(m))
(cl:defmethod r_target-val ((m <GripsTarget>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:r_target-val is deprecated.  Use folding_msgs-msg:r_target instead.")
  (r_target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripsTarget>) ostream)
  "Serializes a message object of type '<GripsTarget>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'l_target) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'r_target) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripsTarget>) istream)
  "Deserializes a message object of type '<GripsTarget>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'l_target) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'r_target) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripsTarget>)))
  "Returns string type for a message object of type '<GripsTarget>"
  "folding_msgs/GripsTarget")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripsTarget)))
  "Returns string type for a message object of type 'GripsTarget"
  "folding_msgs/GripsTarget")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripsTarget>)))
  "Returns md5sum for a message object of type '<GripsTarget>"
  "05ff55cecb9155554097e097eaf570d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripsTarget)))
  "Returns md5sum for a message object of type 'GripsTarget"
  "05ff55cecb9155554097e097eaf570d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripsTarget>)))
  "Returns full string definition for message of type '<GripsTarget>"
  (cl:format cl:nil "GripTarget l_target~%GripTarget r_target~%~%================================================================================~%MSG: folding_msgs/GripTarget~%geometry_msgs/PointStamped point~%string arm~%bool grip~%float64 pitch~%float64 roll~%float64 yaw~%bool empty~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripsTarget)))
  "Returns full string definition for message of type 'GripsTarget"
  (cl:format cl:nil "GripTarget l_target~%GripTarget r_target~%~%================================================================================~%MSG: folding_msgs/GripTarget~%geometry_msgs/PointStamped point~%string arm~%bool grip~%float64 pitch~%float64 roll~%float64 yaw~%bool empty~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripsTarget>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'l_target))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'r_target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripsTarget>))
  "Converts a ROS message object to a list"
  (cl:list 'GripsTarget
    (cl:cons ':l_target (l_target msg))
    (cl:cons ':r_target (r_target msg))
))
