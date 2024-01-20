
(cl:in-package :asdf)

(defsystem "my_package-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "WaypointDistanceRequest" :depends-on ("_package_WaypointDistanceRequest"))
    (:file "_package_WaypointDistanceRequest" :depends-on ("_package"))
    (:file "WaypointDistanceResponse" :depends-on ("_package_WaypointDistanceResponse"))
    (:file "_package_WaypointDistanceResponse" :depends-on ("_package"))
  ))