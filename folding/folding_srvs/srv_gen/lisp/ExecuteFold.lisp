; Auto-generated. Do not edit!


(cl:in-package folding_srvs-srv)


;//! \htmlinclude ExecuteFold-request.msg.html

(cl:defclass <ExecuteFold-request> (roslisp-msg-protocol:ros-message)
  ((fold_traj
    :reader fold_traj
    :initarg :fold_traj
    :type folding_msgs-msg:FoldTraj
    :initform (cl:make-instance 'folding_msgs-msg:FoldTraj)))
)

(cl:defclass ExecuteFold-request (<ExecuteFold-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteFold-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteFold-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<ExecuteFold-request> is deprecated: use folding_srvs-srv:ExecuteFold-request instead.")))

(cl:ensure-generic-function 'fold_traj-val :lambda-list '(m))
(cl:defmethod fold_traj-val ((m <ExecuteFold-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:fold_traj-val is deprecated.  Use folding_srvs-srv:fold_traj instead.")
  (fold_traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteFold-request>) ostream)
  "Serializes a message object of type '<ExecuteFold-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fold_traj) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteFold-request>) istream)
  "Deserializes a message object of type '<ExecuteFold-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fold_traj) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteFold-request>)))
  "Returns string type for a service object of type '<ExecuteFold-request>"
  "folding_srvs/ExecuteFoldRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteFold-request)))
  "Returns string type for a service object of type 'ExecuteFold-request"
  "folding_srvs/ExecuteFoldRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteFold-request>)))
  "Returns md5sum for a message object of type '<ExecuteFold-request>"
  "3e78e4eec8da5701d6f887e332038580")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteFold-request)))
  "Returns md5sum for a message object of type 'ExecuteFold-request"
  "3e78e4eec8da5701d6f887e332038580")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteFold-request>)))
  "Returns full string definition for message of type '<ExecuteFold-request>"
  (cl:format cl:nil "folding_msgs/FoldTraj fold_traj~%~%================================================================================~%MSG: folding_msgs/FoldTraj~%geometry_msgs/PointStamped[] approach_points~%geometry_msgs/PointStamped[] grip_points~%geometry_msgs/PointStamped[] quarter_points~%geometry_msgs/PointStamped[] weight_points~%geometry_msgs/PointStamped[] vertical_points~%geometry_msgs/PointStamped[] goal_points~%geometry_msgs/PointStamped smooth_center~%geometry_msgs/PointStamped[] smooth_edges~%bool ignore_smooth~%float64[] tilts~%bool red~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteFold-request)))
  "Returns full string definition for message of type 'ExecuteFold-request"
  (cl:format cl:nil "folding_msgs/FoldTraj fold_traj~%~%================================================================================~%MSG: folding_msgs/FoldTraj~%geometry_msgs/PointStamped[] approach_points~%geometry_msgs/PointStamped[] grip_points~%geometry_msgs/PointStamped[] quarter_points~%geometry_msgs/PointStamped[] weight_points~%geometry_msgs/PointStamped[] vertical_points~%geometry_msgs/PointStamped[] goal_points~%geometry_msgs/PointStamped smooth_center~%geometry_msgs/PointStamped[] smooth_edges~%bool ignore_smooth~%float64[] tilts~%bool red~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteFold-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fold_traj))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteFold-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteFold-request
    (cl:cons ':fold_traj (fold_traj msg))
))
;//! \htmlinclude ExecuteFold-response.msg.html

(cl:defclass <ExecuteFold-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ExecuteFold-response (<ExecuteFold-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteFold-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteFold-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<ExecuteFold-response> is deprecated: use folding_srvs-srv:ExecuteFold-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ExecuteFold-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:success-val is deprecated.  Use folding_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteFold-response>) ostream)
  "Serializes a message object of type '<ExecuteFold-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteFold-response>) istream)
  "Deserializes a message object of type '<ExecuteFold-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteFold-response>)))
  "Returns string type for a service object of type '<ExecuteFold-response>"
  "folding_srvs/ExecuteFoldResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteFold-response)))
  "Returns string type for a service object of type 'ExecuteFold-response"
  "folding_srvs/ExecuteFoldResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteFold-response>)))
  "Returns md5sum for a message object of type '<ExecuteFold-response>"
  "3e78e4eec8da5701d6f887e332038580")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteFold-response)))
  "Returns md5sum for a message object of type 'ExecuteFold-response"
  "3e78e4eec8da5701d6f887e332038580")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteFold-response>)))
  "Returns full string definition for message of type '<ExecuteFold-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteFold-response)))
  "Returns full string definition for message of type 'ExecuteFold-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteFold-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteFold-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteFold-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ExecuteFold)))
  'ExecuteFold-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ExecuteFold)))
  'ExecuteFold-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteFold)))
  "Returns string type for a service object of type '<ExecuteFold>"
  "folding_srvs/ExecuteFold")
