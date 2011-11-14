; Auto-generated. Do not edit!


(cl:in-package folding_msgs-msg)


;//! \htmlinclude FoldTraj.msg.html

(cl:defclass <FoldTraj> (roslisp-msg-protocol:ros-message)
  ((approach_points
    :reader approach_points
    :initarg :approach_points
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (grip_points
    :reader grip_points
    :initarg :grip_points
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (quarter_points
    :reader quarter_points
    :initarg :quarter_points
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (weight_points
    :reader weight_points
    :initarg :weight_points
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (vertical_points
    :reader vertical_points
    :initarg :vertical_points
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (goal_points
    :reader goal_points
    :initarg :goal_points
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (smooth_center
    :reader smooth_center
    :initarg :smooth_center
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped))
   (smooth_edges
    :reader smooth_edges
    :initarg :smooth_edges
    :type (cl:vector geometry_msgs-msg:PointStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PointStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PointStamped)))
   (ignore_smooth
    :reader ignore_smooth
    :initarg :ignore_smooth
    :type cl:boolean
    :initform cl:nil)
   (tilts
    :reader tilts
    :initarg :tilts
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (red
    :reader red
    :initarg :red
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FoldTraj (<FoldTraj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FoldTraj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FoldTraj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_msgs-msg:<FoldTraj> is deprecated: use folding_msgs-msg:FoldTraj instead.")))

(cl:ensure-generic-function 'approach_points-val :lambda-list '(m))
(cl:defmethod approach_points-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:approach_points-val is deprecated.  Use folding_msgs-msg:approach_points instead.")
  (approach_points m))

(cl:ensure-generic-function 'grip_points-val :lambda-list '(m))
(cl:defmethod grip_points-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:grip_points-val is deprecated.  Use folding_msgs-msg:grip_points instead.")
  (grip_points m))

(cl:ensure-generic-function 'quarter_points-val :lambda-list '(m))
(cl:defmethod quarter_points-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:quarter_points-val is deprecated.  Use folding_msgs-msg:quarter_points instead.")
  (quarter_points m))

(cl:ensure-generic-function 'weight_points-val :lambda-list '(m))
(cl:defmethod weight_points-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:weight_points-val is deprecated.  Use folding_msgs-msg:weight_points instead.")
  (weight_points m))

(cl:ensure-generic-function 'vertical_points-val :lambda-list '(m))
(cl:defmethod vertical_points-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:vertical_points-val is deprecated.  Use folding_msgs-msg:vertical_points instead.")
  (vertical_points m))

(cl:ensure-generic-function 'goal_points-val :lambda-list '(m))
(cl:defmethod goal_points-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:goal_points-val is deprecated.  Use folding_msgs-msg:goal_points instead.")
  (goal_points m))

(cl:ensure-generic-function 'smooth_center-val :lambda-list '(m))
(cl:defmethod smooth_center-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:smooth_center-val is deprecated.  Use folding_msgs-msg:smooth_center instead.")
  (smooth_center m))

(cl:ensure-generic-function 'smooth_edges-val :lambda-list '(m))
(cl:defmethod smooth_edges-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:smooth_edges-val is deprecated.  Use folding_msgs-msg:smooth_edges instead.")
  (smooth_edges m))

(cl:ensure-generic-function 'ignore_smooth-val :lambda-list '(m))
(cl:defmethod ignore_smooth-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:ignore_smooth-val is deprecated.  Use folding_msgs-msg:ignore_smooth instead.")
  (ignore_smooth m))

(cl:ensure-generic-function 'tilts-val :lambda-list '(m))
(cl:defmethod tilts-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:tilts-val is deprecated.  Use folding_msgs-msg:tilts instead.")
  (tilts m))

(cl:ensure-generic-function 'red-val :lambda-list '(m))
(cl:defmethod red-val ((m <FoldTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader folding_msgs-msg:red-val is deprecated.  Use folding_msgs-msg:red instead.")
  (red m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FoldTraj>) ostream)
  "Serializes a message object of type '<FoldTraj>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'approach_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'approach_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grip_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'grip_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'quarter_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'quarter_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'weight_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'weight_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vertical_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'vertical_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goal_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'goal_points))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'smooth_center) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'smooth_edges))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'smooth_edges))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ignore_smooth) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tilts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'tilts))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'red) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FoldTraj>) istream)
  "Deserializes a message object of type '<FoldTraj>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'approach_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'approach_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'grip_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'grip_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'quarter_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'quarter_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'weight_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'weight_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vertical_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vertical_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goal_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goal_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'smooth_center) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'smooth_edges) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'smooth_edges)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PointStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'ignore_smooth) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tilts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tilts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'red) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FoldTraj>)))
  "Returns string type for a message object of type '<FoldTraj>"
  "folding_msgs/FoldTraj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FoldTraj)))
  "Returns string type for a message object of type 'FoldTraj"
  "folding_msgs/FoldTraj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FoldTraj>)))
  "Returns md5sum for a message object of type '<FoldTraj>"
  "34989d2c245d57b9f6a53bb82f486498")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FoldTraj)))
  "Returns md5sum for a message object of type 'FoldTraj"
  "34989d2c245d57b9f6a53bb82f486498")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FoldTraj>)))
  "Returns full string definition for message of type '<FoldTraj>"
  (cl:format cl:nil "geometry_msgs/PointStamped[] approach_points~%geometry_msgs/PointStamped[] grip_points~%geometry_msgs/PointStamped[] quarter_points~%geometry_msgs/PointStamped[] weight_points~%geometry_msgs/PointStamped[] vertical_points~%geometry_msgs/PointStamped[] goal_points~%geometry_msgs/PointStamped smooth_center~%geometry_msgs/PointStamped[] smooth_edges~%bool ignore_smooth~%float64[] tilts~%bool red~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FoldTraj)))
  "Returns full string definition for message of type 'FoldTraj"
  (cl:format cl:nil "geometry_msgs/PointStamped[] approach_points~%geometry_msgs/PointStamped[] grip_points~%geometry_msgs/PointStamped[] quarter_points~%geometry_msgs/PointStamped[] weight_points~%geometry_msgs/PointStamped[] vertical_points~%geometry_msgs/PointStamped[] goal_points~%geometry_msgs/PointStamped smooth_center~%geometry_msgs/PointStamped[] smooth_edges~%bool ignore_smooth~%float64[] tilts~%bool red~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FoldTraj>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'approach_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grip_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'quarter_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'weight_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vertical_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goal_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'smooth_center))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'smooth_edges) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tilts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FoldTraj>))
  "Converts a ROS message object to a list"
  (cl:list 'FoldTraj
    (cl:cons ':approach_points (approach_points msg))
    (cl:cons ':grip_points (grip_points msg))
    (cl:cons ':quarter_points (quarter_points msg))
    (cl:cons ':weight_points (weight_points msg))
    (cl:cons ':vertical_points (vertical_points msg))
    (cl:cons ':goal_points (goal_points msg))
    (cl:cons ':smooth_center (smooth_center msg))
    (cl:cons ':smooth_edges (smooth_edges msg))
    (cl:cons ':ignore_smooth (ignore_smooth msg))
    (cl:cons ':tilts (tilts msg))
    (cl:cons ':red (red msg))
))
