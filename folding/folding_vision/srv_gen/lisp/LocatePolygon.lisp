; Auto-generated. Do not edit!


(cl:in-package folding_vision-srv)


;//! \htmlinclude LocatePolygon-request.msg.html

(cl:defclass <LocatePolygon-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass LocatePolygon-request (<LocatePolygon-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocatePolygon-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocatePolygon-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_vision-srv:<LocatePolygon-request> is deprecated: use folding_vision-srv:LocatePolygon-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocatePolygon-request>) ostream)
  "Serializes a message object of type '<LocatePolygon-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocatePolygon-request>) istream)
  "Deserializes a message object of type '<LocatePolygon-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocatePolygon-request>)))
  "Returns string type for a service object of type '<LocatePolygon-request>"
  "folding_vision/LocatePolygonRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocatePolygon-request)))
  "Returns string type for a service object of type 'LocatePolygon-request"
  "folding_vision/LocatePolygonRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocatePolygon-request>)))
  "Returns md5sum for a message object of type '<LocatePolygon-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocatePolygon-request)))
  "Returns md5sum for a message object of type 'LocatePolygon-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocatePolygon-request>)))
  "Returns full string definition for message of type '<LocatePolygon-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocatePolygon-request)))
  "Returns full string definition for message of type 'LocatePolygon-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocatePolygon-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocatePolygon-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LocatePolygon-request
))
;//! \htmlinclude LocatePolygon-response.msg.html

(cl:defclass <LocatePolygon-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass LocatePolygon-response (<LocatePolygon-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocatePolygon-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocatePolygon-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name folding_vision-srv:<LocatePolygon-response> is deprecated: use folding_vision-srv:LocatePolygon-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocatePolygon-response>) ostream)
  "Serializes a message object of type '<LocatePolygon-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocatePolygon-response>) istream)
  "Deserializes a message object of type '<LocatePolygon-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocatePolygon-response>)))
  "Returns string type for a service object of type '<LocatePolygon-response>"
  "folding_vision/LocatePolygonResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocatePolygon-response)))
  "Returns string type for a service object of type 'LocatePolygon-response"
  "folding_vision/LocatePolygonResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocatePolygon-response>)))
  "Returns md5sum for a message object of type '<LocatePolygon-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocatePolygon-response)))
  "Returns md5sum for a message object of type 'LocatePolygon-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocatePolygon-response>)))
  "Returns full string definition for message of type '<LocatePolygon-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocatePolygon-response)))
  "Returns full string definition for message of type 'LocatePolygon-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocatePolygon-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocatePolygon-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LocatePolygon-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LocatePolygon)))
  'LocatePolygon-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LocatePolygon)))
  'LocatePolygon-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocatePolygon)))
  "Returns string type for a service object of type '<LocatePolygon>"
  "folding_vision/LocatePolygon")
