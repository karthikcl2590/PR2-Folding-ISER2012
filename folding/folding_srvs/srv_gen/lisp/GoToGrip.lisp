; Auto-generated. Do not edit!


(cl:in-package folding_srvs-srv)


;//! \htmlinclude GoToGrip-request.msg.html

(cl:defclass <GoToGrip-request> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type folding_msgs-msg:GripsTarget
    :initform (cl:make-instance 'folding_msgs-msg:GripsTarget)))
)

(cl:defclass GoToGrip-request (<GoToGrip-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToGrip-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToGrip-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<GoToGrip-request> is deprecated: use folding_srvs-srv:GoToGrip-request instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <GoToGrip-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:target-val is deprecated.  Use folding_srvs-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToGrip-request>) ostream)
  "Serializes a message object of type '<GoToGrip-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToGrip-request>) istream)
  "Deserializes a message object of type '<GoToGrip-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToGrip-request>)))
  "Returns string type for a service object of type '<GoToGrip-request>"
  "folding_srvs/GoToGripRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToGrip-request)))
  "Returns string type for a service object of type 'GoToGrip-request"
  "folding_srvs/GoToGripRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToGrip-request>)))
  "Returns md5sum for a message object of type '<GoToGrip-request>"
  "283b0415f491e6b192fcafd3d24dd27c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToGrip-request)))
  "Returns md5sum for a message object of type 'GoToGrip-request"
  "283b0415f491e6b192fcafd3d24dd27c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToGrip-request>)))
  "Returns full string definition for message of type '<GoToGrip-request>"
  (cl:format cl:nil "folding_msgs/GripsTarget target~%~%================================================================================~%MSG: folding_msgs/GripsTarget~%GripTarget l_target~%GripTarget r_target~%~%================================================================================~%MSG: folding_msgs/GripTarget~%geometry_msgs/PointStamped point~%string arm~%bool grip~%float64 pitch~%float64 roll~%float64 yaw~%bool empty~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToGrip-request)))
  "Returns full string definition for message of type 'GoToGrip-request"
  (cl:format cl:nil "folding_msgs/GripsTarget target~%~%================================================================================~%MSG: folding_msgs/GripsTarget~%GripTarget l_target~%GripTarget r_target~%~%================================================================================~%MSG: folding_msgs/GripTarget~%geometry_msgs/PointStamped point~%string arm~%bool grip~%float64 pitch~%float64 roll~%float64 yaw~%bool empty~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToGrip-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToGrip-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToGrip-request
    (cl:cons ':target (target msg))
))
;//! \htmlinclude GoToGrip-response.msg.html

(cl:defclass <GoToGrip-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GoToGrip-response (<GoToGrip-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoToGrip-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoToGrip-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<GoToGrip-response> is deprecated: use folding_srvs-srv:GoToGrip-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GoToGrip-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:success-val is deprecated.  Use folding_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoToGrip-response>) ostream)
  "Serializes a message object of type '<GoToGrip-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoToGrip-response>) istream)
  "Deserializes a message object of type '<GoToGrip-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoToGrip-response>)))
  "Returns string type for a service object of type '<GoToGrip-response>"
  "folding_srvs/GoToGripResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToGrip-response)))
  "Returns string type for a service object of type 'GoToGrip-response"
  "folding_srvs/GoToGripResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoToGrip-response>)))
  "Returns md5sum for a message object of type '<GoToGrip-response>"
  "283b0415f491e6b192fcafd3d24dd27c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoToGrip-response)))
  "Returns md5sum for a message object of type 'GoToGrip-response"
  "283b0415f491e6b192fcafd3d24dd27c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoToGrip-response>)))
  "Returns full string definition for message of type '<GoToGrip-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoToGrip-response)))
  "Returns full string definition for message of type 'GoToGrip-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoToGrip-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoToGrip-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GoToGrip-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GoToGrip)))
  'GoToGrip-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GoToGrip)))
  'GoToGrip-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoToGrip)))
  "Returns string type for a service object of type '<GoToGrip>"
  "folding_srvs/GoToGrip")
