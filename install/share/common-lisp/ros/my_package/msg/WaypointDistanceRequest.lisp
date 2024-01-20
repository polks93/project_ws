; Auto-generated. Do not edit!


(cl:in-package my_package-msg)


;//! \htmlinclude WaypointDistanceRequest.msg.html

(cl:defclass <WaypointDistanceRequest> (roslisp-msg-protocol:ros-message)
  ((robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:fixnum
    :initform 0)
   (waypoint_id
    :reader waypoint_id
    :initarg :waypoint_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass WaypointDistanceRequest (<WaypointDistanceRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointDistanceRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointDistanceRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_package-msg:<WaypointDistanceRequest> is deprecated: use my_package-msg:WaypointDistanceRequest instead.")))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <WaypointDistanceRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-msg:robot_id-val is deprecated.  Use my_package-msg:robot_id instead.")
  (robot_id m))

(cl:ensure-generic-function 'waypoint_id-val :lambda-list '(m))
(cl:defmethod waypoint_id-val ((m <WaypointDistanceRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_package-msg:waypoint_id-val is deprecated.  Use my_package-msg:waypoint_id instead.")
  (waypoint_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointDistanceRequest>) ostream)
  "Serializes a message object of type '<WaypointDistanceRequest>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waypoint_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointDistanceRequest>) istream)
  "Deserializes a message object of type '<WaypointDistanceRequest>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'waypoint_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointDistanceRequest>)))
  "Returns string type for a message object of type '<WaypointDistanceRequest>"
  "my_package/WaypointDistanceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointDistanceRequest)))
  "Returns string type for a message object of type 'WaypointDistanceRequest"
  "my_package/WaypointDistanceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointDistanceRequest>)))
  "Returns md5sum for a message object of type '<WaypointDistanceRequest>"
  "831c18e1bdee664e0ac4ccc25433d3c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointDistanceRequest)))
  "Returns md5sum for a message object of type 'WaypointDistanceRequest"
  "831c18e1bdee664e0ac4ccc25433d3c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointDistanceRequest>)))
  "Returns full string definition for message of type '<WaypointDistanceRequest>"
  (cl:format cl:nil "uint8    robot_id~%uint8    waypoint_id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointDistanceRequest)))
  "Returns full string definition for message of type 'WaypointDistanceRequest"
  (cl:format cl:nil "uint8    robot_id~%uint8    waypoint_id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointDistanceRequest>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointDistanceRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointDistanceRequest
    (cl:cons ':robot_id (robot_id msg))
    (cl:cons ':waypoint_id (waypoint_id msg))
))
