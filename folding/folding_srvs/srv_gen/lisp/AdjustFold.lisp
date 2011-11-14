; Auto-generated. Do not edit!


(cl:in-package folding_srvs-srv)


;//! \htmlinclude AdjustFold-request.msg.html

(cl:defclass <AdjustFold-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (end
    :reader end
    :initarg :end
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped)))
)

(cl:defclass AdjustFold-request (<AdjustFold-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AdjustFold-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AdjustFold-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<AdjustFold-request> is deprecated: use folding_srvs-srv:AdjustFold-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <AdjustFold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:start-val is deprecated.  Use folding_srvs-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <AdjustFold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:end-val is deprecated.  Use folding_srvs-srv:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AdjustFold-request>) ostream)
  "Serializes a message object of type '<AdjustFold-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AdjustFold-request>) istream)
  "Deserializes a message object of type '<AdjustFold-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AdjustFold-request>)))
  "Returns string type for a service object of type '<AdjustFold-request>"
  "folding_srvs/AdjustFoldRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AdjustFold-request)))
  "Returns string type for a service object of type 'AdjustFold-request"
  "folding_srvs/AdjustFoldRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AdjustFold-request>)))
  "Returns md5sum for a message object of type '<AdjustFold-request>"
  "6b200e0e09c9fc34fb9a2a20661850b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AdjustFold-request)))
  "Returns md5sum for a message object of type 'AdjustFold-request"
  "6b200e0e09c9fc34fb9a2a20661850b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AdjustFold-request>)))
  "Returns full string definition for message of type '<AdjustFold-request>"
  (cl:format cl:nil "geometry_msgs/PointStamped start~%geometry_msgs/PointStamped end~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AdjustFold-request)))
  "Returns full string definition for message of type 'AdjustFold-request"
  (cl:format cl:nil "geometry_msgs/PointStamped start~%geometry_msgs/PointStamped end~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AdjustFold-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AdjustFold-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AdjustFold-request
    (cl:cons ':start (start msg))
    (cl:cons ':end (end msg))
))
;//! \htmlinclude AdjustFold-response.msg.html

(cl:defclass <AdjustFold-response> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (end
    :reader end
    :initarg :end
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped)))
)

(cl:defclass AdjustFold-response (<AdjustFold-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AdjustFold-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AdjustFold-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<AdjustFold-response> is deprecated: use folding_srvs-srv:AdjustFold-response instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <AdjustFold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:start-val is deprecated.  Use folding_srvs-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <AdjustFold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:end-val is deprecated.  Use folding_srvs-srv:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AdjustFold-response>) ostream)
  "Serializes a message object of type '<AdjustFold-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AdjustFold-response>) istream)
  "Deserializes a message object of type '<AdjustFold-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AdjustFold-response>)))
  "Returns string type for a service object of type '<AdjustFold-response>"
  "folding_srvs/AdjustFoldResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AdjustFold-response)))
  "Returns string type for a service object of type 'AdjustFold-response"
  "folding_srvs/AdjustFoldResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AdjustFold-response>)))
  "Returns md5sum for a message object of type '<AdjustFold-response>"
  "6b200e0e09c9fc34fb9a2a20661850b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AdjustFold-response)))
  "Returns md5sum for a message object of type 'AdjustFold-response"
  "6b200e0e09c9fc34fb9a2a20661850b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AdjustFold-response>)))
  "Returns full string definition for message of type '<AdjustFold-response>"
  (cl:format cl:nil "geometry_msgs/PointStamped start~%geometry_msgs/PointStamped end~%~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AdjustFold-response)))
  "Returns full string definition for message of type 'AdjustFold-response"
  (cl:format cl:nil "geometry_msgs/PointStamped start~%geometry_msgs/PointStamped end~%~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AdjustFold-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AdjustFold-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AdjustFold-response
    (cl:cons ':start (start msg))
    (cl:cons ':end (end msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AdjustFold)))
  'AdjustFold-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AdjustFold)))
  'AdjustFold-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AdjustFold)))
  "Returns string type for a service object of type '<AdjustFold>"
  "folding_srvs/AdjustFold")
