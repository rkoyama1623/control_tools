(let ( (robot-name (unix:getenv "ROBOT")) )
  (cond
   ( (equal robot-name "HRP2JSK"   )
     (warning-message 3 "loading robot interface file...~%")
     (require :hrp2jsk-interface "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
     (setq *robot-class* hrp2jsk-robot     *hrp2jsk*     (instance *robot-class* :init) *robot* *hrp2jsk*))
   ( (equal robot-name "HRP2JSKNT" )
     (warning-message 3 "loading robot interface file...~%")
     (require :hrp2jsknt-interface "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
     (setq *robot-class* hrp2jsknt-robot   *hrp2jsknt*   (instance *robot-class* :init) *robot* *hrp2jsknt*))
   ( (equal robot-name "HRP2JSKNTS" )
     (warning-message 3 "loading robot interface file...~%")
     (require :hrp2jsknts-interface "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
     (setq *robot-class* hrp2jsknts-robot  *hrp2jsknts*  (instance *robot-class* :init) *robot* *hrp2jsknts*))
   ( (equal robot-name "JAXON" )
     (warning-message 3 "loading robot interface file...~%")
     (require :jaxon-interface "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon-interface.l")
     (setq *robot-class* jaxon-robot  *jaxon*  (instance *robot-class* :init) *robot* *jaxon*))
   ( (equal robot-name "JAXON_RED" )
     (warning-message 3 "loading robot interface file...~%")
     (require :jaxon_red-interface "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
     (setq *robot-class* jaxon_red-robot  *jaxon_red*  (instance *robot-class* :init) *robot* *jaxon_red*))
   ( (equal robot-name "TESTMDOFARM")
     (warning-message 3 "loading robot interface file...~%")
     (require :testmdofarm-interface "package://hrpsys_ros_bridge_tutorials/euslisp/testmdofarm-interface.l")
     (setq *robot-class* testmdofarm-robot *testmdofarm* (instance *robot-class* :init) *robot* *testmdofarm*))
   ( t (warning-message 1 "!!!!!!!!!!No ROBOT TYPE!!!!!!!!!!~%") )
   ))

;; disable needless module loading
(unless (assoc :hrp2jsk-interface     *modules*) (nconc *modules* (list (list* :hrp2jsk-interface     nil))))
(unless (assoc :hrp2jsknt-interface   *modules*) (nconc *modules* (list (list* :hrp2jsknt-interface   nil))))
(unless (assoc :hrp2jsknts-interface  *modules*) (nconc *modules* (list (list* :hrp2jsknts-interface  nil))))
(unless (assoc :urataleg-interface    *modules*) (nconc *modules* (list (list* :urataleg-interface    nil))))
(unless (assoc :staro-interface       *modules*) (nconc *modules* (list (list* :staro-interface       nil))))
(unless (assoc :jaxon-interface       *modules*) (nconc *modules* (list (list* :jaxon-interface       nil))))
(unless (assoc :jaxon_red-interface   *modules*) (nconc *modules* (list (list* :jaxon_red-interface   nil))))
(unless (assoc :samplerobot-interface *modules*) (nconc *modules* (list (list* :samplerobot-interface nil))))

;; for log-parse
;; (require "package://hrpsys_ros_bridge/euslisp/datalogger-log-parser.l")
;; (require "euslib/irteus_proposals/motion-lib-proposal.l")
(warning-message 3 "loading test-st-state-estimate-data.l...~%")
(require (format nil "~a/ros/~a/src/jsk-ros-pkg/euslib/demo/nozawa/hrpsys-test/test-st-state-estimate-data.l" (unix:getenv "HOME") (unix:getenv "ROS_DISTRO")))

(defvar *usage-funcs* nil)
(unless (assoc :hrpsys-base-log-parser *usage-funcs*)
  (push (cons :hrpsys-base-log-parser
              #'(lambda ()
                  (warn "~%hrpsys-base-log-parser.l~%")
                  (warning-message 1 "Execute in log dir!!~%")
                  (warning-message 1 "Or log file name has to correspond to log directory name!!~%")
                  (warning-message 3 "(prepare-plot :robot hrp2jsknts-robot)~%")
                  (warning-message 3 "(plot-rs :mode :ee-pos :axis 2 :proc-mode :press-enter :yrange (list 0 0.1) :robot hrp2jsknts-robot :save t :create t)~%")
                  (warning-message 3 "(plot-imu :fname \"HRP2JSKNTS/go-vel-test_hoge\" :axis 1 :start-tm 7 :stop-tm 8 :robot hrp2jsknts-robot :save t :create t :correct-offset t)~%")
                  (warn "~%"))) *usage-funcs*))

(defun create-log-interface (fname-with-path &key (robot))
  (instance datalogger-log-parser-controller :init fname-with-path :robot robot) )

(defun create-rs-list (&key (ri *log*))
  ;; (setq *rs-list* (convert-to-robot-state-data-from-ri :ri *log*))
  (convert-to-robot-state-data-for-st-debug)
  (add-data-to-rs-list)
  *rs-list*
  )

;; デフォルトではない値を*rs-list*に追加
(defun add-data-to-rs-list ()
  (let* ((ret) (init-tm) (idx 0)
         (limb-list
          (mapcar #'(lambda (x) (find-if #'(lambda (l) (member x (send (send *log* :robot) l :force-sensors))) '(:rleg :lleg :rarm :larm)))
                  (send (send *log* :robot) :force-sensors)))
         (limb-idx (let ((cnt -1)) (mapcar #'(lambda (x) (incf cnt)) limb-list))))
    (do-until-key-with-check
     (progn (null-output (send *log* :state)) (if (find-method *log* :eof-p) (not (send *log* :eof-p))))
     (unless init-tm
       (setq init-tm (send *log* :time)))
     ;; use reference from st, instead of sh
     (send *log* :set-robot-state1
           :reference-vector
           ;; (map float-vector #'rad2deg (send (send self :parser-list "sh_q") :read-state)))
           (map float-vector #'rad2deg (send (send *log* :parser-list "sh_qOut") :read-state)))
     ;; (map float-vector #'rad2deg (send (send *log* :parser-list "sh_q") :read-state)))
     (let ((rs (elt *rs-list* idx)))
       (send rs :put :sh-lleg-ref-force (send (send *log* :parser-list "sh_lfsensorOut") :read-state))
       (send rs :put :sh-rleg-ref-force (send (send *log* :parser-list "sh_rfsensorOut") :read-state))
       (send rs :put :sh-larm-ref-force (send (send *log* :parser-list "sh_lhsensorOut") :read-state))
       (send rs :put :sh-rarm-ref-force (send (send *log* :parser-list "sh_rhsensorOut") :read-state))
       (setf (elt *rs-list* idx) rs)
       )
     (incf idx)
     )
    (length *rs-list*)))

(defun plot-rs (&key (fname nil) (mode :root-rot) (axis 2) (limb nil) (proc-mode :sleep) (robot hrp2jsknts-robot)
                     (save nil) (xrange nil) (yrange nil) (create nil) (rs-list *rs-list*))
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq rs-list (car xrange) (cadr xrange)))
         fname-with-path start-idx stop-idx )

    (setq fname-with-path (prepare-plot :fname fname :create create :robot robot));; rs-list生成 fname-with-pathをparse

    (setq start-idx 0 stop-idx (length *rs-list*))

    ;; (when xrange
    ;;   (setq dt (calc-dt *rs-list*) start-idx (round (/ (car xrange) dt)) stop-idx (round (/ (cadr xrange) dt)) ))


    ;; (send *gp* :command "set grid;set size 1,1")
    (if yrange (send *gp* :command (format nil " set yrange [~a:~a]" (car yrange) (cdr yrange))))
    ;; (plot-robot-state-data-list (subseq *rs-list* start-idx stop-idx) :mode mode :axis axis :limb limb :robot (send *log* :robot) :proc-mode proc-mode)
    (plot-robot-state-data-list data :mode mode :axis axis :limb limb :robot (send *log* :robot) :proc-mode proc-mode)

    (when save
      ;; (send *gp* :command "set grid;set size 2,1")
      ;; (plot-robot-state-data-list (subseq *rs-list* start-idx stop-idx) :mode mode :axis axis :limb limb
      ;;                             :robot (send *log* :robot) :proc-mode :sleep :fname (concatenate string fname-with-path "_"))
      (plot-robot-state-data-list data :mode mode :axis axis :limb limb
                                  :robot (send *log* :robot) :proc-mode :sleep :fname (concatenate string fname-with-path "_"))

      ;; (mapc #'(lambda (val) (if (reg-match ".eps" val) (unix:system (format nil "convert ~s ~s" val (reg-replace ".eps" ".png" val))))) (directory))
      )

    ;; (unix:system "beep")
    ))

(defun plot-imu (&key (fname nil) (axis 1) (proc-mode :sleep) (save nil) (yrange nil) 
                      (robot hrp2jsknts-robot) (create nil) (correct-offset nil)
                      (start-tm nil) (stop-tm nil))
  (let* ( (imu-msg) (init-tm)
         (ylabel "rot [deg]") 
         fname-with-path
         (gname (format nil "chest-rpy ~A ~A" (case axis (0 "y") (1 "p") (2 "r")) (read-from-string ylabel))))

    (prepare-plot :fname fname :create create :robot robot :create create);; rs-list生成
    (setq tm-list (mapcar #'(lambda (x) (send x :time)) *rs-list*))

    (if (not (boundp '*gp*)) (setq *gp* (gnuplot)))

    (cond
     ( (equal robot hrp2jsknts-robot)
       (if (not (boundp '*hrp2jsknts*)) (hrp2jsknts-init))
       (setq *robot* *hrp2jsknts*) )
     ( (equal robot testmdofarm-robot)
       (if (not (boundp '*testmdofarm*)) (testmdofarm-init))
       (setq *robot* *testmdofarm*) )
     (t (warning-message 1 "Robot-instance not defined"))
     )

    ;; create imu list
    ;; (when (or (not (boundp 'true-p-list)) (not (boundp 'sensor-p-list)) create)
      (setq sensor-p-list nil  true-p-list nil)
      (warning-message 3 "now creating imu list~%")
      (do-until-key-with-check
       (progn (send *log* :state) (if (find-method *log* :eof-p) (not (send *log* :eof-p))))
       (progn
         (unless init-tm (setq init-tm (send *log* :time)))

         ;; (car (rpy-angle (quaternion2matrix (send (send *log* :imu) :orientation))))
         ;; (send *robot* :newcoords (make-coords)) (send *robot* :angle-vector (send *log* :potentio-vector)) (car (rpy-angle (car (send *robot* :imu-sensors))))

         (setq imu-msg (send (cdr (assoc :imu ( *log* . robot-state))) :orientation))
         (push-back 
          (rad2deg (elt (car (rpy-angle 
                              ;; (quaternion2matrix (float-vector  (send imu-msg :w) (send imu-msg :x) (send imu-msg :y) (send imu-msg :z) ))
                              (ros::tf-quaternion->rot imu-msg)
                              )) axis))
          sensor-p-list)

         (send *robot* :newcoords (make-coords)) (send *robot* :angle-vector (send *log* :potentio-vector))
         (push-back (rad2deg (elt (car (rpy-angle (send  (car (send *robot* :imu-sensors)) :worldrot))) axis)) true-p-list)
         ))
      (when correct-offset (setq true-p-list (mapcar #'(lambda (val) (+ val (car sensor-p-list))) true-p-list)))
      ;; )
    
    (setq start-tm (if start-tm start-tm (car tm-list))
          stop-tm  (if stop-tm  stop-tm  (car (last tm-list))) )

    ;; (send *gp* :command "set xlabel \"Time[sec]\"") (send *gp* :command "set ylabel \"rot [deg]\"")
    ;; (send *gp* :command "set tics font \"Helvetica,10\"")
    ;; (send *gp* :command "set xlabel font \"Helvetica,10\"") (send *gp* :command "set ylabel font \"Helvetica,10\"")
    (send *gp* :command "set grid;set size 1,1")
    (graph-view-with-filter (list true-p-list sensor-p-list) tm-list 
                            :keylist '("true" "sensor")
                            :xlabel "Time [s]"  :ylabel ylabel
                            :xrange (list start-tm stop-tm) :yrange yrange
                            :title gname
                            )

    (when save
      (send *gp* :command "set grid;set size 2,1")
      (graph-view-with-filter (list true-p-list sensor-p-list) tm-list 
                              :keylist '("true" "senseor")
                              :xlabel "Time [s]" :ylabel ylabel
                              :xrange (list start-tm stop-tm) :yrange yrange
                              :title gname
                              :dump-graph t
                              :graph-fname (format nil "~A~A.eps" fname-with-path (reg-replace* " " "_" gname))
                              )
      (mapc #'(lambda (val) (if (reg-match ".eps" val) (unix:system (format nil "convert ~a ~a" val (reg-replace ".eps" ".png" val))))) (directory))
      )

    ))

(defun plot-mu
  (&key (rs-list *rs-list*)
        (add-fname nil) (fname "Friction Coefficient")
        (cutoff-f nil) (xrange nil) (limb '(:lleg :rleg)) (save t) )
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq rs-list (car xrange) (cadr xrange))))
    (graph-view
     (delete nil 
             (list (if (find :rleg limb)
                       (low-pass-filter (mapcar #'(lambda (x) (/ (norm (subseq (send x :get :rleg-force-vector) 0 2)) (elt (send x :get :rleg-force-vector) 2))) data)
                                        :cutoff-f cutoff-f))
                   (if (find :lleg limb)
                       (low-pass-filter (mapcar #'(lambda (x) (/ (norm (subseq (send x :get :lleg-force-vector) 0 2)) (elt (send x :get :lleg-force-vector) 2))) data) 
                                        :cutoff-f cutoff-f))))
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (delete nil (list (if (find :rleg limb) "Right Foot(stop)") (if (find :lleg limb) "Left Foot(slipping)")))
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph save
     :xlabel "Time[s]"
     :ylabel "Friction Coefficient"
     )
    ))

(defun calc-dt (target-list)
  (let ( (accuracy 0.001) )
    (* (round (/ (- (send (cadr target-list) :time) (send (car target-list) :time)) accuracy)) accuracy)
  ))

(defun plot-foot-force
  (&key (rs-list *rs-list*)
        (add-fname nil) (fname "Actual and Reference and Modified Reference Force")
        (axis 2) (cutoff-f nil) (xrange nil) (limb '(:lleg :rleg)) (save t))
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq *rs-list* (car xrange) (cadr xrange))))
    (graph-view
     (delete nil (list (if (find :rleg limb) (low-pass-filter (mapcar #'(lambda (x) (elt (send x :get :rleg-force-vector) axis)) data) :cutoff-f cutoff-f))
                       (if (find :lleg limb) (low-pass-filter (mapcar #'(lambda (x) (elt (send x :get :lleg-force-vector) axis)) data) :cutoff-f cutoff-f))
                       (if (find :rleg limb) (mapcar #'(lambda (x) (elt (send x :get :sh-rleg-ref-force) axis)) data))
                       (if (find :lleg limb) (mapcar #'(lambda (x) (elt (send x :get :sh-lleg-ref-force) axis)) data))
                       (if (find :rleg limb) (mapcar #'(lambda (x) (elt (send x :get :st-rleg-ref-force) axis)) data))
                       (if (find :lleg limb) (mapcar #'(lambda (x) (elt (send x :get :st-lleg-ref-force) axis)) data))))
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (delete nil (list (if (find :rleg limb) "act rleg force z") (if (find :lleg limb) "act lleg force z")
                                (if (find :rleg limb) "ref rleg force z") (if (find :lleg limb) "ref lleg force z")
                                (if (find :rleg limb) "modified ref rleg force z") (if (find :lleg limb) "modified ref lleg force z") ))
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph save
     :xlabel "Time[s]"
     :ylabel "Foot force [N]"
     )
    ))

(defun calc-r (alpha-zmp)
  (let ( (steepness 8) )
    ;; (+ (/ -1 (+ 1 (exp (* -6 steepness (+ alpha-zmp -1 (/ 1.0 steepness))))))  (/ 1 (+ 1 (exp (* -6 steepness (- alpha-zmp (/ 1.0 steepness)))))))
    (+ (/ 1 (+ 1 (exp (+ (* -48 alpha-zmp) 6)))) (/ -1 (+ 1 (exp (+ (* -48 alpha-zmp) 42)))))
  ))

(defun plot-alpha
  (&key (rs-list *rs-list*) (xrange nil) (axis 0)
        (add-fname nil) (fname (format nil "state data ~A" add-fname))
        (cutoff-f nil) (save t)
        (leg-margin 65))
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq *rs-list* (car xrange) (cadr xrange)))
          (zmpy) (redge) (alpha-zmp) (r) (lleg-ref-force-z) (rleg-ref-force-z) (alpha-sfc) (alpha)
          (alpha-zmp-list nil) (alpha-sfc-list nil) (r-list nil) (alpha-list nil)
          (foot-edge 0) )
    (mapc #'(lambda (x) 
              (send *robot* :angle-vector (send x :angle-vector))
              (setq zmpy (* 1e3 (elt (send x :get :reference-zmp) 1))
                    redge (+ (elt (send (send *robot* :rleg :end-coords) :worldpos) 1) leg-margin) ;; rleg=stop
                    alpha-zmp (/ (max (- zmpy redge) 0) (- (- (elt (send (send *robot* :lleg :end-coords) :worldpos) 1) leg-margin) redge))
                    r (calc-r alpha-zmp)
                    lleg-ref-force-z (elt (send x :get :sh-lleg-ref-force) 2)
                    rleg-ref-force-z (elt (send x :get :sh-rleg-ref-force) 2)
                    alpha-sfc (/ lleg-ref-force-z (+ lleg-ref-force-z rleg-ref-force-z))
                    alpha (+ (* r alpha-sfc) (* (- 1 r) alpha-zmp)))
              (push-back alpha-zmp alpha-zmp-list)
              (push-back alpha-sfc alpha-sfc-list)
              (push-back r r-list)
              (push-back alpha alpha-list)
              ) data)
    (graph-view
     (list (low-pass-filter alpha-zmp-list :cutoff-f cutoff-f)
           ;; (low-pass-filter r-list :cutoff-f cutoff-f)
           ;; (low-pass-filter alpha-sfc-list :cutoff-f cutoff-f)
           ;; (low-pass-filter alpha-list :cutoff-f cutoff-f)
           ;; (mapcar #'(lambda (x) (* 1e3 (- (elt (send x :get :reference-zmp) 1) foot-edge))) data)
           )
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (list "alpha-zmp" "r" "alpha-sfc" "alpha")
     ;; :keylist (list "refzmp y" "refzmp y from edge")
     :title fname
     :xlabel "Time [s]" :ylabel "ZMP, COG[mm]"
     :dump-graph save
     )
    ))
