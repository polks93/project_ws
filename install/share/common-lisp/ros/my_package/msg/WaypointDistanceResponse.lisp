; Auto-generated. Do not edit!


(cl:in-package my_package-msg)


;//! \htmlinclude WaypointDistanceResponse.msg.html

(cl:defclass <WaypointDistanceResponse> (roslisp-msg-protocol:ros-message)
  ((robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:fixnum
    :initform 0)
   (waypoint_id
    :reader waypoint_id
    :initarg :waypoint_id
    :type cl:fixnum
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass WaypointDistanceResponse (<WaypointDistanceResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointDistanceResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointDistanceResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_package-msg:<WaypointDistanceResponse> is deprecated: use my_package-msg:WaypointDistanceResponse instead.")))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <WaypointDistanceResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-msg:robot_id-val is deprecated.  Use my_package-msg:robot_id instead.")
  (robot_id m))

(cl:ensure-generic-function 'waypoint_id-val :lambda-list '(m))
(cl:defmethod waypoint_id-val ((m <WaypointDistanceResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-msg:waypoint_id-val is deprecated.  Use my_package-msg:waypoint_id instead.")
  (waypoint_id m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <WaypointDistanceResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-msg:distance-val is deprecated.  Use my_package-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointDistanceResponse>) ostream)
  "Serializes a message object of type '<WaypointDistanceResponse>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waypoint_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointDistanceResponse>) istream)
  "Deserializes a message object of type '<WaypointDistanceResponse>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waypoint_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointDistanceResponse>)))
  "Returns string type for a message object of type '<WaypointDistanceResponse>"
  "my_package/WaypointDistanceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointDistanceResponse)))
  "Returns string type for a message object of type 'WaypointDistanceResponse"
  "my_package/WaypointDistanceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointDistanceResponse>)))
  "Returns md5sum for a message object of type '<WaypointDistanceResponse>"
  "29d251301658634b82c36b56fb83e302")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointDistanceResponse)))
  "Returns md5sum for a message object of type 'WaypointDistanceResponse"
  "29d251301658634b82c36b56fb83e302")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointDistanceResponse>)))
  "Returns full string definition for message of type '<WaypointDistanceResponse>"
  (cl:format cl:nil "uint8       robot_id~%uint8       waypoint_id~%float32     distance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointDistanceResponse)))
  "Returns full string definition for message of type 'WaypointDistanceResponse"
  (cl:format cl:nil "uint8       robot_id~%uint8       waypoint_id~%float32     distance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointDistanceResponse>))
  (cl:+ 0
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointDistanceResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointDistanceResponse
    (cl:cons ':robot_id (robot_id msg))
    (cl:cons ':waypoint_id (waypoint_id msg))
    (cl:cons ':distance (distance msg))
))
