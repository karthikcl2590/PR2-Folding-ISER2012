; Auto-generated. Do not edit!


(cl:in-package folding_srvs-srv)


;//! \htmlinclude FoldingStance-request.msg.html

(cl:defclass <FoldingStance-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FoldingStance-request (<FoldingStance-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FoldingStance-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FoldingStance-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<FoldingStance-request> is deprecated: use folding_srvs-srv:FoldingStance-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FoldingStance-request>) ostream)
  "Serializes a message object of type '<FoldingStance-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FoldingStance-request>) istream)
  "Deserializes a message object of type '<FoldingStance-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FoldingStance-request>)))
  "Returns string type for a service object of type '<FoldingStance-request>"
  "folding_srvs/FoldingStanceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FoldingStance-request)))
  "Returns string type for a service object of type 'FoldingStance-request"
  "folding_srvs/FoldingStanceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FoldingStance-request>)))
  "Returns md5sum for a message object of type '<FoldingStance-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FoldingStance-request)))
  "Returns md5sum for a message object of type 'FoldingStance-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FoldingStance-request>)))
  "Returns full string definition for message of type '<FoldingStance-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FoldingStance-request)))
  "Returns full string definition for message of type 'FoldingStance-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FoldingStance-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FoldingStance-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FoldingStance-request
))
;//! \htmlinclude FoldingStance-response.msg.html

(cl:defclass <FoldingStance-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FoldingStance-response (<FoldingStance-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FoldingStance-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FoldingStance-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_srvs-srv:<FoldingStance-response> is deprecated: use folding_srvs-srv:FoldingStance-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FoldingStance-response>) ostream)
  "Serializes a message object of type '<FoldingStance-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FoldingStance-response>) istream)
  "Deserializes a message object of type '<FoldingStance-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FoldingStance-response>)))
  "Returns string type for a service object of type '<FoldingStance-response>"
  "folding_srvs/FoldingStanceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FoldingStance-response)))
  "Returns string type for a service object of type 'FoldingStance-response"
  "folding_srvs/FoldingStanceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FoldingStance-response>)))
  "Returns md5sum for a message object of type '<FoldingStance-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FoldingStance-response)))
  "Returns md5sum for a message object of type 'FoldingStance-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FoldingStance-response>)))
  "Returns full string definition for message of type '<FoldingStance-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FoldingStance-response)))
  "Returns full string definition for message of type 'FoldingStance-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FoldingStance-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FoldingStance-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FoldingStance-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FoldingStance)))
  'FoldingStance-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FoldingStance)))
  'FoldingStance-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FoldingStance)))
  "Returns string type for a service object of type '<FoldingStance>"
  "folding_srvs/FoldingStance")
