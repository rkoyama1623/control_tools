(require "package://roseus_tutorials/src/kinect-model-viewer.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(load "~/ros/indigo_parent/src/jskeus/irteus/irtrobot.l")

(defun rosbag-play () (unix:system "sh -c 'rosbag play ~/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/human/test/only-tf_2015-10-11-13-06-43.bag'&"))

#-:ros
(unless (find-package "ROS") (make-package "ROS"))
#+:ros
(defun play-motion (&key robot fname loop-hook (max-user-index 5) (time-span 0.5) (real nil)) ;; :fname "test-kinect.bvh", loop-hook is a function to call inside do-until-key loop
  (let ((floor (make-cube 2000 1000 10 :pos #f(1000 0 0))) f)
    (if fname (setq f (open fname :direction :output)))
    (ros::roseus "kinect_bvh")
    (if (not (boundp '*tl*))
        (setq *tl* (instance ros::transform-listener :init)))
    (setq *model* (make-kinect-bvh-robot-model)) ;; Kinect Human Model
    (objects (list floor *model*))
    (if robot (objects robot))
    ;;(defun find-node (name) (find name *arrows* :key #'(lambda (x) (send x :name)) :test #'string=))
    (if f (send *model* :dump-hierarchy f))
    ;;;
    ;;; Main Loop
    ;;;
    (do-until-key
     (let ()
       (when (send *model* :copy-state-from-tf *tl* :max-user-index max-user-index) ;; copy kinect human tracker information to human body
         ;;
         ;; track human body
         ;;
         ;; (format t "kinect robot : larm pos=~A, rarm pos=~A (world)~%" ;; display arm tool coordinates in world coordinates
         ;;         (send *model* :larm :end-coords :worldpos)
         ;;         (send *model* :rarm :end-coords :worldpos))
         ;; (format t "             : larm pos=~A, rarm pos=~A (local)~%" ;; display arm tool cooridnates with reference to human torso coordinates
         ;;         (send (send (send *model* :torso :end-coords) :transformation (send *model* :larm :end-coords)) :worldpos)
         ;;         (send (send (send *model* :torso :end-coords) :transformation (send *model* :rarm :end-coords)) :worldpos))
         (when robot
           ;; (send *model* :copy-state-to robot)
           (let ( (cur-time (car (unix:ptimes))) )
             (when (> (- cur-time prev-time) (* 100 time-span))
               (setq prev-time cur-time)
               (send *model* :copy-state-to robot)
               (when real (send *ri* :angle-vector (send robot :angle-vector) time-span))
               )
             )
           );; robot

         )
       (if (functionp loop-hook) (funcall loop-hook))
       (send (get *viewer* :pickviewer) :look-all)
       (x::window-main-one)
       (if f (send *model* :dump-motion f))
       ) ;; do-until-key
      )
    ))
