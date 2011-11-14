; Auto-generated. Do not edit!


(cl:in-package folding_srvs-srv)


;//! \htmlinclude GetTable-request.msg.html

(cl:defclass <GetTable-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetTable-request (<GetTable-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTable-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTable-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<GetTable-request> is deprecated: use folding_srvs-srv:GetTable-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTable-request>) ostream)
  "Serializes a message object of type '<GetTable-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTable-request>) istream)
  "Deserializes a message object of type '<GetTable-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTable-request>)))
  "Returns string type for a service object of type '<GetTable-request>"
  "folding_srvs/GetTableRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTable-request)))
  "Returns string type for a service object of type 'GetTable-request"
  "folding_srvs/GetTableRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTable-request>)))
  "Returns md5sum for a message object of type '<GetTable-request>"
  "79c0eb4705f19651b0b57a66d8c9a156")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTable-request)))
  "Returns md5sum for a message object of type 'GetTable-request"
  "79c0eb4705f19651b0b57a66d8c9a156")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTable-request>)))
  "Returns full string definition for message of type '<GetTable-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTable-request)))
  "Returns full string definition for message of type 'GetTable-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTable-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTable-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTable-request
))
;//! \htmlinclude GetTable-response.msg.html

(cl:defclass <GetTable-response> (roslisp-msg-protocol:ros-message)
  ((shape
    :reader shape
    :initarg :shape
    :type geometric_shapes_msgs-msg:Shape
    :initform (cl:make-instance 'geometric_shapes_msgs-msg:Shape))
   (pose_stamped
    :reader pose_stamped
    :initarg :pose_stamped
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass GetTable-response (<GetTable-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTable-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTable-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<GetTable-response> is deprecated: use folding_srvs-srv:GetTable-response instead.")))

(cl:ensure-generic-function 'shape-val :lambda-list '(m))
(cl:defmethod shape-val ((m <GetTable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:shape-val is deprecated.  Use folding_srvs-srv:shape instead.")
  (shape m))

(cl:ensure-generic-function 'pose_stamped-val :lambda-list '(m))
(cl:defmethod pose_stamped-val ((m <GetTable-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_srvs-srv:pose_stamped-val is deprecated.  Use folding_srvs-srv:pose_stamped instead.")
  (pose_stamped m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTable-response>) ostream)
  "Serializes a message object of type '<GetTable-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'shape) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_stamped) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTable-response>) istream)
  "Deserializes a message object of type '<GetTable-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'shape) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_stamped) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTable-response>)))
  "Returns string type for a service object of type '<GetTable-response>"
  "folding_srvs/GetTableResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTable-response)))
  "Returns string type for a service object of type 'GetTable-response"
  "folding_srvs/GetTableResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTable-response>)))
  "Returns md5sum for a message object of type '<GetTable-response>"
  "79c0eb4705f19651b0b57a66d8c9a156")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTable-response)))
  "Returns md5sum for a message object of type 'GetTable-response"
  "79c0eb4705f19651b0b57a66d8c9a156")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTable-response>)))
  "Returns full string definition for message of type '<GetTable-response>"
  (cl:format cl:nil "geometric_shapes_msgs/Shape shape~%geometry_msgs/PoseStamped pose_stamped~%~%~%================================================================================~%MSG: geometric_shapes_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTable-response)))
  "Returns full string definition for message of type 'GetTable-response"
  (cl:format cl:nil "geometric_shapes_msgs/Shape shape~%geometry_msgs/PoseStamped pose_stamped~%~%~%================================================================================~%MSG: geometric_shapes_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTable-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'shape))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_stamped))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTable-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTable-response
    (cl:cons ':shape (shape msg))
    (cl:cons ':pose_stamped (pose_stamped msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTable)))
  'GetTable-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTable)))
  'GetTable-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTable)))
  "Returns string type for a service object of type '<GetTable>"
  "folding_srvs/GetTable")
