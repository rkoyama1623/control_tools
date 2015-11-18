(require "package://roseus_tutorials/src/kinect-model-viewer.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(load "~/ros/indigo_parent/src/jskeus/irteus/irtrobot.l")

(ros::roseus "real-time-skeleton-operation.l")
(ros::load-ros-manifest "gazebo_msgs")

(ros::rate 45)

(when (not (boundp '*pickview*))
  (setq *pickview* (pickview :no-menu t))
  (send *pickview* :resize 300 300))

(defvar font-helvetica-20 (x::font-id "-*-Helvetica-Medium-R-Normal-*-20-*"))

;; (defun rosbag-play () (unix:system "sh -c 'rosbag play ~/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/human/test/only-tf_2015-10-11-13-06-43.bag'&"))
(defun rosbag-play () (unix:system "sh -c 'rosparam set use_sim_time true; rosbag play -s 20 ~/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/human/test/kinect2_2015-10-31-22-58-23.bag --clock'&"))

#-:ros
(unless (find-package "ROS") (make-package "ROS"))
#+:ros
(defun play-motion (&key robot fname loop-hook (show-all t) (stop 100) (max-user-index 5) (time-span 0.5) (real nil) (bag nil)) ;; :fname "test-kinect.bvh", loop-hook is a function to call inside do-until-key loop
  (let ((floor (make-cube 2000 1000 10 :pos #f(1000 0 0))) f)
    (if bag (rosbag-play))
    (if fname (setq f (open fname :direction :output)))
    (ros::roseus "kinect_bvh")
    (if (not (boundp '*tl*))
        (setq *tl* (instance ros::transform-listener :init)))
    (setq *model* (make-kinect-bvh-robot-model)) ;; Kinect Human Model
    (if show-all (objects (list floor *model*)) (objects *model*))
		(send *model* :angle-vector (concatenate float-vector #f(4000 0 1000) (make-array 30  :initial-element 0 :element-type 'float)))
		(send (get *viewer* :pickviewer) :look-all)
    (if (and robot show-all) (objects robot))
    ;;(defun find-node (name) (find name *arrows* :key #'(lambda (x) (send x :name)) :test #'string=))
    (if f (send *model* :dump-hierarchy f))
    ;;;
    ;;; Main Loop
    ;;;
    (let ( (prev-time (car (unix:ptimes))) )
      (warning-message 3 "started~%")
      (do-until-key
       (when (send *model* :copy-state-from-tf *tl* :max-user-index max-user-index) ;; copy kinect human tracker information to human body
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
               ;; (send *hrp2jsknt* :rarm :elbow-p :joint-angle (- (rad2deg (acos (apply #'v. (send-all (send-all (subseq (send *model* :links) 7   9) :worldcoords) :axis :x))))))
               ;; (send *hrp2jsknt* :larm :elbow-p :joint-angle (- (rad2deg (acos (apply #'v. (send-all (send-all (subseq (send *model* :links) 10 12) :worldcoords) :axis :x))))))
               (set-arm-joint-angle)
               ;; (copy-joint-position :stop stop)
               ;; (send robot :newcoords (make-coords :pos (send *model* :worldpos) :rot (ros::tf-quaternion->rot (send (elt (send *msg* :pose) 1) :orientation))))
							 (send robot :move-coords (make-coords :pos (send *model* :worldpos)
																										 :rot (ros::tf-quaternion->rot (send (elt (send *msg* :pose) 13) :orientation)))
										 (elt (send robot :links) 15))
               
               ;; foot ik
               (mapc #'(lambda (limb)
                         (send *hrp2jsknt* limb 
                               :inverse-kinematics (make-coords)
                               :move-target (send *hrp2jsknt* limb :end-coords)
                               :weight #f(0 0 0 0 1 1 0)  
                               :translation-axis nil :rotation-axis :z)) '(:lleg :rleg))

               (when real (send *ri* :angle-vector (send robot :angle-vector) time-span))
               )
             )
           );; robot

         )

       (if (functionp loop-hook) (funcall loop-hook))
       ;; (send (get *viewer* :pickviewer) :look-all)
			 (print-manual)
			 (send *pickview* :viewer :viewsurface :flush)
			 (send *pickview* :draw-objects :flush nil)
       (ros::spin-once)
       (x::window-main-one)
       (if f (send *model* :dump-motion f))
       ) ;; do-until-key
      )
    ))

;; (ros::subscribe "hoge" std_msgs::string
;;                 #'(lambda (msg) (ros::ros-info 
;;                                  (format nil "I heard ~A" (send msg :data)))))

(ros::subscribe "gazebo/model_states"  gazebo_msgs::modelstates #'(lambda (msg) (setq *msg* msg)))

;; (ros::subscribe "tf_old" tf2_msgs::tfmessage
;;                 #'(lambda (msg) 
;;                     (let ( (tf-stamped (car (send msg :transforms))) )
;;                       ;; (format t "header time:~a id:~a~%" (send (send tf-stamped :header :stamp) :sec-nsec) (send tf-stamped :header :frame_id))
;;                       ;; (format t "child_frame_id ~a~%" (send tf-stamped :child_frame_id))
;;                       ;; (format t "transform ~a~%" (send tf-stamped :transform))
;;                       ;; (format t "pose ~a~%" (send (send tf-stamped :transform) :slots))

;;                       (when (substringp "left_hip" (send tf-stamped :child_frame_id))
;;                         (format t "child_frame_id ~a~%" (send tf-stamped :child_frame_id))
;;                         ;; (let ( (tf (send tf-stamped :transform)) )
;;                         ;;   (send *box* :newcoords
;;                         ;;         (make-coords
;;                         ;;          :pos (scale 1000 (float-vector (send tf :translation :x) (send tf :translation :y) (send tf :translation :z)))
;;                         ;;          ;; :rot (user::quaternion2matrix (float-vector (send tf :rotation :w) (send tf :rotation :x) (send tf :rotation :y) (send tf :rotation :z)))
;;                         ;;          ))
;;                         ;;   )
;;                         )

;;                       ;; (setq ret (ros::eustf-lookup-transform cobject
;;                       ;;                                        target-frame source-frame
;;                       ;;                                        (send time :sec-nsec)))
;;                       )
;;                     ))

;; (if (not (boundp '*box*)) (setq *box* (make-cube 10 100 300)))
;; (objects (list *box* (make-cube 10 10 10)))

;; (ros::rate 1000) ;; ??
;; (sys:thread-no-wait #'(lambda () (do-until-key (ros::spin-once))))

;; (do-until-key 
;;  (ros::spin-once))
 
;; (do-until-key 
;;  (objects)
;;  (x::window-main-one)
;;  )


(defun copy-joint-position (&key (stop 100))
  (let ()
    ;; (send *model* :angle-vector #f(2182.29 -156.567 1676.39 -177.948 63.0659 -89.9343   10.906 0.049403 -2.21981 -3.036 1.00219 -8.91678  -10.2811 3.9249 -45.7732 -59.1707 2.37302 -91.4156 58.4171 56.0704 1.07593 -5.412783e-07 -4.388677e-06 114.499 -26.9467 -2.1042 -0.036513 -0.956893 0.090992 -1.48031 0.090065 0.479401 -0.000377))

    ;; for OpenNI
    ;; (setq torso-coord (send (send (send (send (car (send *model* :links)) :worldcoords) :copy-worldcoords) :rotate  (deg2rad -90) :x :local) :rotate (deg2rad 90) :z :local))
    ;; for windows
    (setq torso-coord (send (send (send (send (car (send *model* :links)) :worldcoords) :copy-worldcoords) :rotate  (deg2rad -90) :x :local) :rotate (deg2rad -90) :z :local))

    (send *hrp2jsknt* :move-coords torso-coord (car (send *hrp2jsknt* :links)))
    ;; (setq model-limb-list (subseq (send *model* :links) 1 13))
    ;; (setq robot-limb-list (send *hrp2jsknt* :lleg))

    ;; for OpenNI
    ;; (setq joint-list '((:torso :waist-p) 
    ;;                    (:rleg :knee-p) (:rleg :ankle-p) (:lleg :knee-p) (:lleg :ankle-p)
    ;;                    (:rarm :shoulder-y) (:rarm :wrist-y) (:larm :shoulder-y) (:larm :wrist-y)))
    ;; for windows
    (setq joint-list '((:torso :waist-p) 
                       (:lleg :knee-p) (:lleg :ankle-p) (:rleg :knee-p) (:rleg :ankle-p)
                       (:larm :shoulder-y) (:larm :wrist-y) (:rarm :shoulder-y) (:rarm :wrist-y)))

    ;; 各関節にcascoordsを追加
    (mapc #'(lambda (joint)
              (send (send *hrp2jsknt* (car joint) (cadr joint) :parent-link)  :assoc (make-cascoords :coords (send *hrp2jsknt* (car joint) (cadr joint) :parent-link :worldcoords :copy-worldcoords) :name (cadr joint)))) joint-list)
    ;; 追加したcascoordsをmove-targetに設定
    (setq move-targets (mapcar #'(lambda (joint) (car (send (send (send *hrp2jsknt* (car joint) (cadr joint)) :parent-link) :descendants))) joint-list))
    (setq link-lists (mapcar #'(lambda (move-target) (send *hrp2jsknt* :link-list (send move-target :parent))) move-targets))

    (setq target-coords (flatten (list (mapcar #'(lambda (link) (send (make-coords :coords (send link :worldcoords)) :rotate (deg2rad -90) :x)) (subseq (send *model* :links) 1 3))
                                       (mapcar #'(lambda (link) (send (make-coords :coords (send link :worldcoords)) :rotate (deg2rad -90) :x)) (subseq (send *model* :links) 4 6))
                                       (mapcar #'(lambda (link) (send (make-coords :coords (send link :worldcoords)) :rotate (deg2rad -90) :y)) (subseq (send *model* :links) 7 9))
                                       (mapcar #'(lambda (link) (send (make-coords :coords (send link :worldcoords)) :rotate (deg2rad 90) :y)) (subseq (send *model* :links) 10 12))
                                       )))
    (push torso-coord target-coords)

    (send *hrp2jsknt* :fullbody-inverse-kinematics 
          target-coords
          :move-target move-targets
          :link-list link-lists
          :translation-axis '(nil  nil nil nil nil  nil nil nil nil)
          :rotation-axis '(nil :z :z :z :z  :z :z :z :z)
          :rthre '(10  20 20 20 20  20 20 20 20)
          :root-link-virtual-joint-weight #f(10 10 10 10 10 10)
          :additional-weight-list (list (list (send *hrp2jsknt* :torso :waist-p :parent-link) 0)
                                        (list (send *hrp2jsknt* :torso :waist-y :parent-link) 0))
          :stop stop
          ;; :debug-view :no-message
          )
    (send *hrp2jsknt* :move-coords torso-coord (car (send *hrp2jsknt* :links)))

    ))

(defun set-arm-joint-angle ()
  ;; (setq waist-coord (make-coords :rpy (car (send (car (send *model* :torso)) :rpy-angle))))
  (setq waist-coord (send (send (send (send (car (send *model* :links)) :worldcoords) :copy-worldcoords) :rotate  (deg2rad -90) :x :local) :rotate (deg2rad 90) :z :local))
  (setq av (send *model* :angle-vector))
  (setq larm-pos-list (send-all (subseq (send *model* :links) 10 13) :worldpos));; 左手
  (setq rarm-pos-list (send-all (subseq (send *model* :links) 7  10) :worldpos));; 右手

  (send *hrp2jsknt* :rarm :angle-vector (calc-arm-joint-angle rarm-pos-list waist-coord :rarm))
  (send *hrp2jsknt* :larm :angle-vector (calc-arm-joint-angle larm-pos-list waist-coord :larm))
  ;; (send *model* :angle-vector
  ;;       (concatenate float-vector
  ;;                    (scale 1000 (subseq av 0 3))
  ;;                    (subseq av 3 18)
  ;;                    (calc-arm-joint-angle rarm-pos-list waist-coord :rarm)
  ;;                    (calc-arm-joint-angle larm-pos-list waist-coord :larm)
  ;;                    #f(0 0 0)
  ;;                    ))
  )

(defun calc-arm-joint-angle (arm-pos-list coord limb)
   (let (q0 q1 q2 q3 q4 q5 q6 y yul u u_ l h h_ a_ (q3-thresh (deg2rad 10)) rate elbow-axis-sign z_  R-yxz-y )
     ;; 体中心座標系に変更
     (dotimes (i (length arm-pos-list)) (setf (elt arm-pos-list i) (send coord :inverse-rotate-vector (elt arm-pos-list i))))

     ;; (setq arm-pos-list (project-to-regression-plane arm-pos-list (car arm-pos-list)));; 回帰平面に投影

     ;; 左右対応
     (cond
      ( (equal limb :larm) (setq z_ #f(0 0 -1) ) )
      ( (equal limb :rarm) (setq z_ #f(0 0  1) ) )
      ( t (warning-message 1 "limb is void~%") )
      )

     (setq u (normalize-vector (v- (elt arm-pos-list 1) (elt arm-pos-list 0)))
           l (normalize-vector (v- (elt arm-pos-list 2) (elt arm-pos-list 1)))
           ;; h (normalize-vector (v- (elt arm-pos-list 3) (elt arm-pos-list 2)))
           )

     (setq yul (normalize-vector (v* u l)))

     ;; (setq elbow-axis-sign (/ (v. yfl yul) (abs (v. yfl yul))));; 肘関節の回転方向
     (setq elbow-axis-sign 1)
     ;; (setq yul (scale  elbow-axis-sign yu));; 膝が逆関節の時はyulを逆向きにする

     ;; q3
     (setq q3 (* elbow-axis-sign (* -1 (acos (v. u l)))))

     ;; q0
     ;; q1が90度近くの時のことを考慮しなくていい??
     (setq q0 (atan (* -1 (elt u 0)) (* -1 (elt u 2))))

     ;; q1
     (setq u_ (transform (inverse-matrix (rotation-matrix q0 :y)) u))
     (setq q1 (atan (elt u_ 1) (* -1 (elt u_ 2))))

     ;; q2
     (cond
      ( (eps= (sin q1) 0.0 1.0e-6)
        (if (eps= (cos q0) 0.0 1.0e-6) (setq q2 (asin (/ (elt yul 2) (sin q0) -1))) (setq q2 (asin (/ (elt yul 0) (cos q0))))) )
      ( t (setq q2 (asin (- (* (cos q0) (elt yul 0)) (* (sin q0) (elt yul 2))))) )
      )

     ;; 肘が伸びきっている場合
     (when (< (abs q3) q3-thresh)
       (setq rate (/ (abs q3) q3-thresh))
       (setq q2 (* rate q2))
       )

     ;; q6
     (setq q6 0)

     ;; 以下要改良 wrist-yawの動きが速すぎる
     ;; h_
     ;; (setq R-yxz-y (reduce #'m* (list (rotation-matrix q0 :y) (rotation-matrix q1 :x) (rotation-matrix q2 :z) (rotation-matrix q3 :y))) )
     ;; (setq h_ (transform (inverse-matrix R-yxz-y) h))


     ;; (setq q4 (atan (elt h_ 0) (* -1 (elt h_ 1))))
     ;; (cond
     ;;  ( (> q4 (deg2rad 90))  (setq q4 (- q4 (deg2rad 180))) )
     ;;  ( (< q4 (deg2rad -90)) (setq q4 (+ q4 (deg2rad 180))) )
     ;;  )
     
     ;; ;; q5
     ;; (cond
     ;;  ( (eps= (cos q4) 0 1.0e-6) (setq q5 (asin (/ (elt h_ 1) (cos q4)))) )
     ;;  ( t (setq q5 (asin (/ (elt h_ 0) (sin q4) -1))) )
     ;;  )

     ;; ;; 手首が伸びきっている場合
     ;; (when (< (abs q5) q3-thresh)
     ;;   (setq rate (/ (abs q5) q3-thresh))
     ;;   (setq q4 (* rate q4))
     ;;   )
     ;; (setq q4 0 q5 0)

     ;; (format stdout "~a~%"
     ;;         (float-vector (rad2deg q2) (rad2deg q1) (rad2deg q0) (rad2deg q3) 0 0 (rad2deg q4) (rad2deg q5) (rad2deg q6)))
     (float-vector (rad2deg q0) (rad2deg q1) (rad2deg q2);; y x z(bvhの定義順)
                   (rad2deg q3) 0 0 
                   0 0 ;; for hrp2
                   ;; (rad2deg q4) (rad2deg q5) (rad2deg q6)
                   )
     ))

(send *pickview* :viewer :viewsurface :color #f(1 1 1))
(defun print-manual ()
	(send *pickview* :viewer :viewsurface :string 0 22 "Move in front of the screen." font-helvetica-20)
	(send *pickview* :viewer :viewsurface :string 0 44 "Robot imitates your pose." font-helvetica-20)
	(send *pickview* :viewer :viewsurface :string 0 66 "Try to stand up." font-helvetica-20)
)
