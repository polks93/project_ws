;; Auto-generated. Do not edit!


(when (boundp 'my_package::WaypointDistanceRequest)
  (if (not (find-package "MY_PACKAGE"))
    (make-package "MY_PACKAGE"))
  (shadow 'WaypointDistanceRequest (find-package "MY_PACKAGE")))
(unless (find-package "MY_PACKAGE::WAYPOINTDISTANCEREQUEST")
  (make-package "MY_PACKAGE::WAYPOINTDISTANCEREQUEST"))

(in-package "ROS")
;;//! \htmlinclude WaypointDistanceRequest.msg.html


(defclass my_package::WaypointDistanceRequest
  :super ros::object
  :slots (_robot_id _waypoint_id ))

(defmethod my_package::WaypointDistanceRequest
  (:init
   (&key
    ((:robot_id __robot_id) 0)
    ((:waypoint_id __waypoint_id) 0)
    )
   (send-super :init)
   (setq _robot_id (round __robot_id))
   (setq _waypoint_id (round __waypoint_id))
   self)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:waypoint_id
   (&optional __waypoint_id)
   (if __waypoint_id (setq _waypoint_id __waypoint_id)) _waypoint_id)
  (:serialization-length
   ()
   (+
    ;; uint8 _robot_id
    1
    ;; uint8 _waypoint_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _robot_id
       (write-byte _robot_id s)
     ;; uint8 _waypoint_id
       (write-byte _waypoint_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _robot_id
     (setq _robot_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _waypoint_id
     (setq _waypoint_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get my_package::WaypointDistanceRequest :md5sum-) "831c18e1bdee664e0ac4ccc25433d3c2")
(setf (get my_package::WaypointDistanceRequest :datatype-) "my_package/WaypointDistanceRequest")
(setf (get my_package::WaypointDistanceRequest :definition-)
      "uint8    robot_id
uint8    waypoint_id
")



(provide :my_package/WaypointDistanceRequest "831c18e1bdee664e0ac4ccc25433d3c2")


