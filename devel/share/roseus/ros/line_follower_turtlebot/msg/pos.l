;; Auto-generated. Do not edit!


(when (boundp 'line_follower_turtlebot::pos)
  (if (not (find-package "LINE_FOLLOWER_TURTLEBOT"))
    (make-package "LINE_FOLLOWER_TURTLEBOT"))
  (shadow 'pos (find-package "LINE_FOLLOWER_TURTLEBOT")))
(unless (find-package "LINE_FOLLOWER_TURTLEBOT::POS")
  (make-package "LINE_FOLLOWER_TURTLEBOT::POS"))

(in-package "ROS")
;;//! \htmlinclude pos.msg.html


(defclass line_follower_turtlebot::pos
  :super ros::object
  :slots (_direction ))

(defmethod line_follower_turtlebot::pos
  (:init
   (&key
    ((:direction __direction) 0.0)
    )
   (send-super :init)
   (setq _direction (float __direction))
   self)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:serialization-length
   ()
   (+
    ;; float64 _direction
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _direction
       (sys::poke _direction (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _direction
     (setq _direction (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get line_follower_turtlebot::pos :md5sum-) "027b9297308a5b7b690348b04b1998f1")
(setf (get line_follower_turtlebot::pos :datatype-) "line_follower_turtlebot/pos")
(setf (get line_follower_turtlebot::pos :definition-)
      "float64 direction

")



(provide :line_follower_turtlebot/pos "027b9297308a5b7b690348b04b1998f1")


