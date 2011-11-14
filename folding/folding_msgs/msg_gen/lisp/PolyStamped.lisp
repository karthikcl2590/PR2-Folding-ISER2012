; Auto-generated. Do not edit!


(cl:in-package folding_msgs-msg)


;//! \htmlinclude PolyStamped.msg.html

(cl:defclass <PolyStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (vertices
    :reader vertices
    :initarg :vertices
    :type (cl:vector folding_msgs-msg:Point2D)
   :initform (cl:make-array 0 :element-type 'folding_msgs-msg:Point2D :initial-element (cl:make-instance 'folding_msgs-msg:Point2D)))
   (z_offset
    :reader z_offset
    :initarg :z_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass PolyStamped (<PolyStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolyStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolyStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_msgs-msg:<PolyStamped> is deprecated: use folding_msgs-msg:PolyStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PolyStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:header-val is deprecated.  Use folding_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'vertices-val :lambda-list '(m))
(cl:defmethod vertices-val ((m <PolyStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:vertices-val is deprecated.  Use folding_msgs-msg:vertices instead.")
  (vertices m))

(cl:ensure-generic-function 'z_offset-val :lambda-list '(m))
(cl:defmethod z_offset-val ((m <PolyStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:z_offset-val is deprecated.  Use folding_msgs-msg:z_offset instead.")
  (z_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolyStamped>) ostream)
  "Serializes a message object of type '<PolyStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vertices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'vertices))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolyStamped>) istream)
  "Deserializes a message object of type '<PolyStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vertices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vertices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'folding_msgs-msg:Point2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_offset) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolyStamped>)))
  "Returns string type for a message object of type '<PolyStamped>"
  "folding_msgs/PolyStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolyStamped)))
  "Returns string type for a message object of type 'PolyStamped"
  "folding_msgs/PolyStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolyStamped>)))
  "Returns md5sum for a message object of type '<PolyStamped>"
  "52bf82a1af6df03ccce898a721fbe1d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolyStamped)))
  "Returns md5sum for a message object of type 'PolyStamped"
  "52bf82a1af6df03ccce898a721fbe1d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolyStamped>)))
  "Returns full string definition for message of type '<PolyStamped>"
  (cl:format cl:nil "Header header~%Point2D[] vertices~%float64 z_offset~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: folding_msgs/Point2D~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolyStamped)))
  "Returns full string definition for message of type 'PolyStamped"
  (cl:format cl:nil "Header header~%Point2D[] vertices~%float64 z_offset~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: folding_msgs/Point2D~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolyStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vertices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolyStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'PolyStamped
    (cl:cons ':header (header msg))
    (cl:cons ':vertices (vertices msg))
    (cl:cons ':z_offset (z_offset msg))
))
