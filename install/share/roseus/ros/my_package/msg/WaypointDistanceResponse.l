;; Auto-generated. Do not edit!


(when (boundp 'my_package::WaypointDistanceResponse)
  (if (not (find-package "MY_PACKAGE"))
    (make-package "MY_PACKAGE"))
  (shadow 'WaypointDistanceResponse (find-package "MY_PACKAGE")))
(unless (find-package "MY_PACKAGE::WAYPOINTDISTANCERESPONSE")
  (make-package "MY_PACKAGE::WAYPOINTDISTANCERESPONSE"))

(in-package "ROS")
;;//! \htmlinclude WaypointDistanceResponse.msg.html


(defclass my_package::WaypointDistanceResponse
  :super ros::object
  :slots (_robot_id _waypoint_id _distance ))

(defmethod my_package::WaypointDistanceResponse
  (:init
   (&key
    ((:robot_id __robot_id) 0)
    ((:waypoint_id __waypoint_id) 0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _robot_id (round __robot_id))
   (setq _waypoint_id (round __waypoint_id))
   (setq _distance (float __distance))
   self)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:waypoint_id
   (&optional __waypoint_id)
   (if __waypoint_id (setq _waypoint_id __waypoint_id)) _waypoint_id)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; uint8 _robot_id
    1
    ;; uint8 _waypoint_id
    1
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _robot_id
       (write-byte _robot_id s)
     ;; uint8 _waypoint_id
       (write-byte _waypoint_id s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _robot_id
     (setq _robot_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _waypoint_id
     (setq _waypoint_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_package::WaypointDistanceResponse :md5sum-) "29d251301658634b82c36b56fb83e302")
(setf (get my_package::WaypointDistanceResponse :datatype-) "my_package/WaypointDistanceResponse")
(setf (get my_package::WaypointDistanceResponse :definition-)
      "uint8       robot_id
uint8       waypoint_id
float32     distance
")



(provide :my_package/WaypointDistanceResponse "29d251301658634b82c36b56fb83e302")


