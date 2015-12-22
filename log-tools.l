(if (unix:getenv "OLD_HRPSYS")
    (progn ;; 旧システム用
      (warning-message 1 "!!!!!!!!!!using old hrpsys!!!!!!!!!!!~%")
      (load "hrpsys-old-log-parser.l")
      ;; (when (not (boundp '*gnuplot*))
      ;;   (setq *gnuplot* (gnuplot) *gplot* *gnuplot* *gp* *gnuplot*)
      ;;   (send *gnuplot* :command "set grid")(send *gnuplot* :command "set size 1,1"))
      )
  (progn ;; 新システム用
    (load "hrpsys-base-log-parser.l")
    ;; (when (not (boundp '*gp*))
    ;;   (setq *gp* (gnuplot)) (send *gp* :command "set grid") )
    )
  )

(load (format nil "~a/ros/~a/src/jsk-ros-pkg/euslib/jsk/gnuplotlib.l" (unix:getenv "HOME") (unix:getenv "ROS_DISTRO")))

(defvar *usage-funcs* nil)
(unless (assoc :log-tools *usage-funcs*)
  (push (cons :log-tools
              #'(lambda ()
                  (warn "~%log-tools.l~%")
                  (warning-message 3 "(save-log :fname <file-name> :add-fname <file-name-option> :type <:new or :old>)~%")
                  (warning-message 3 "(make-list-from-input :fname <motion-name>  :span <time-span> )~%")
                  (warning-message 3 "(make-list-and-file-from-log :start <start-index> :span <time-span>)~%")
                  (warning-message 3 "(save-list '(<list-name1> <list-name2>) :make-script t :type <log-type> :multi-plot nil)~%")
                  (warning-message 3 "joint velocity and torque list name: joint-vel-tq-list~%")
                  (warn "~%"))) *usage-funcs*))

(cond
 ( (boundp '*gnuplot*) (setq *gp* *gnuplot*) )
 ( (boundp '*gp*) (setq *gnuplot** *gp*) )
 ( t
   (setq *gnuplot* (gnuplot) *gplot* *gnuplot* *gp* *gnuplot*)
   (send *gnuplot* :command "set grid")(send *gnuplot* :command "set size 1,1") )
 )

(defun save-log (&key (fname nil) (add-fname nil) (type :new) (param-list nil) (parse-flg nil))
  ;; デフォルトログファイル名
  (if (not fname) (setq fname *__fname__*))
  (if (and add-fname (not (equal add-fname ""))) (setq fname (format nil "~a_~a" fname add-fname)))

  ;; param-listをファイル名に反映
  (push "" param-list)
  (setq fname (concatenate
               string fname (reduce #'(lambda (x y) (reg-replace "\\." "-" (concatenate string  x "_" (string (car y)) (cadr y))))  param-list)))

  (cond
   ( (equal type :old) (save-log-old fname) )
   ( (equal type :new) (setq fname (save-log-new fname parse-flg)) )
   )
  fname
  )
;; プラグインのログを保存してローカルにコピー
;; 旧システム
(defun save-log-old (fname)
  (let ( machine-name raw-date (date-list nil) path )

    (setq raw-date (unix:localtime))
    ;; 年 月 日 時(前後1時間のログをコピーする:hrp2内部時刻のPCの時刻のズレは1時間以下でなければならない)
    (push (reg-replace* " " "0" (format nil "~2d~2d~2d~2d" (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (1- (elt raw-date 2)) )) date-list)
    (push (reg-replace* " " "0" (format nil "~2d~2d~2d~2d" (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3)     (elt raw-date 2)  )) date-list)
    (push (reg-replace* " " "0" (format nil "~2d~2d~2d~2d" (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (1+ (elt raw-date 2)) )) date-list)

    (send *ci* :stop-and-save-log fname)

    (cond
     ( (equal (send *robot* :name) "HRP2JSK")    (setq machine-name "hrp2007c") )
     ( (equal (send *robot* :name) "HRP2JSKNT")  (setq machine-name "hrp2016c") )
     ( (equal (send *robot* :name) "HRP2JSKNTS") (setq machine-name "hrp2017c") )
     ( t (warning-message 1 "cannot use save-log in this robot~%") )
     )

    (mapc #'(lambda (val)
              (unix:system (concatenate string "scp grxuser@" machine-name ":/tmp/" fname "-log-" machine-name "-" val "*.log /tmp/" ))) date-list)

    (setq path (format nil "/home/~a/Copy/Documents/log/~a/~a-log-~a-20~d_~d_~d_~d_~d_~d"
                       (unix:getenv "USER") (send *robot* :name) fname machine-name
                       (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (elt raw-date 2) (elt raw-date 1) (elt raw-date 0) ) )
    (unix:system (format nil "mkdir -p ~a" path))
    (unix:system (format nil "find /tmp -name ~a* 2>/dev/null | head -7 | xargs -i cp {} ~a" (concatenate string fname "-log-") path))

    (send *ci* :start-log) ))

;; 新システム
(defun save-log-new (fname parse-flg)
  (let ( machine-name raw-date date path sim-flg fstream uname (org-fname "test") )

    ;; 日付追加
    (setq raw-date (unix:localtime))
    (setq date (reg-replace* " " "0" (format nil "20~2d_~2d-~2d_~2d-~2d_~2d" (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (elt raw-date 2) (elt raw-date 1) (elt raw-date 0)  )) )

    (setq org-fname (format nil "~a_~a" org-fname date))
    (send *ri* :save-log (format nil "/tmp/~a" org-fname) :set-robot-date-string nil)


    (setq fname (format nil "~a_~a_~a" fname (send *robot* :name) date))

    (setq sim-flg (ros::get-param "use_sim_time" nil));; シミュレーションフラグ
    
		;; コピー先設定
    (setq path (format nil "/home/~a/k-kojima@jsk.imi.i.u-tokyo.ac.jp/Documents/log/~a/~a"
                       (unix:getenv "USER") (send *robot* :name) fname) )
    (unix:system (format nil "mkdir -p ~a" path))

    ;; コピー
    (cond
     ( sim-flg 
       ;; (setq uname (unix:getenv "USER")
			 ;; 			 cp-command "cp"
			 ;; 			 machine-name (unix:system "uname -n"))
       ;; (with-open-file (fstream "/etc/hostname" :direction :input)
       ;;                 (let ( (eof (gensym)) ) (setq machine-name (read-line fstream nil eof)) ))
			 (unix:system (concatenate string "cp /tmp/" org-fname "* " path))
			 )
     ( t
       (setq uname "grxuser" cp-command "scp")
       (cond
        ( (equal (send *robot* :name) "HRP2JSK")    (setq machine-name "hrp2007c") )
        ( (equal (send *robot* :name) "HRP2JSKNT")  (setq machine-name "hrp2016c") )
        ( (equal (send *robot* :name) "HRP2JSKNTS") (setq machine-name "hrp2017c") )
        ( t (warning-message 1 "cannot use save-log in this robot~%") )
        )
			 (unix:system (concatenate string "scp " uname "@" machine-name ":/tmp/" org-fname "* " path))
			 )
     )


    ;; (sys:thread-no-wait #'plot-rs :fname fname :mode :force :save t :create t)
    (if parse-flg (sys:thread-no-wait #'prepare-plot :fname fname :create t))
    ;; (sys:thread-no-wait #'plot-mu :fname fname :save t)

    fname
    ))

;; プロットをアニメーションで表示
(defun anim_gnuplot(&key (yrange "[-3:3]") (xtics 0.5) (ylabel "joint angular velocity [rad/s]")
                         (fname "./logs/tmplog.dat")
                         (indices (list 2))
                         (with "line")  
                         (speed 5) (tm-width 2) (offset-tm 1) (start-tm 0) (stop-tm 10) (data-span 3)
                         )
  (let (tmpstr (dt 0.005) )
    (send *gnuplot* :command "set grid")
    (send *gnuplot* :command "set xlabel \"time[s]\"")
    (send *gnuplot* :command (format nil "set ylabel \"~a\"" ylabel))
    (send *gnuplot* :command (format nil "set yrange ~a" yrange))
    (send *gnuplot* :command (format nil "set xtics ~a" xtics))
    (send *gnuplot* :command (format nil "set parametric;set trange ~a;" yrange))
    
    (dotimes (j (round (/ (- stop-tm start-tm) dt)))
      (when (= (mod j speed) 0)
        (send *gnuplot* :command (format nil "set xrange [~a:~a]" (+ start-tm (* j dt) (- offset-tm)) (+ start-tm (* j dt) (- offset-tm) tm-width)))
        (send *gnuplot* :command (format nil "const=~a;" (+ start-tm (* j dt))))

        (if (= j 0)
            (progn
              (setq tmpstr "plot const,t notitle;")
              (dolist (index indices)
                (setq tmpstr (concatenate string tmpstr (format nil "replot \"~a\" every ~a::::~a using 1:~a with ~a title columnhead;"
                                                                fname data-span (round (/ stop-tm dt)) index  with))) )
              (send *gnuplot* :command tmpstr)
              )
          (send *gnuplot* :command "replot")
          )
        )
      
      )
    )
  )

(defun parse-fname-with-path (fname);; fnameは変更されない
  (let ( (fname-with-path) )
    (if fname
        (progn 
          (case (probe-file fname)
            (:file (setq fname-with-path (car (reg-split "." fname))) )                       ;; fnameにログファイル名を指定した場合
            (:directory (setq fname-with-path (concatenate string fname "/" (car (last (reg-split "/" fname)))) )) ;; fnameにログディレクトリ名を指定した場合
            (nil (setq fname-with-path 
                       (concatenate string (unix:getenv "HOME") "/Copy/Documents/log/" (send *robot* :name) "/" fname "/" (car (last (reg-split "/" fname))))) )
            )
          ;; (setq fname (car (last (reg-split "/" fname))))
          )
      (setq fname (car (last (reg-split "/" (unix::getenv "PWD"))));; path名とfnameが同じと仮定
            fname-with-path (concatenate string (unix::getenv "PWD") "/" fname))
      )
    fname-with-path
    ))

(defun parse-fname (fname);; fnameは変更されない
  (if fname
      (setq fname (car (last (reg-split "/" fname))))
    (setq fname (car (last (reg-split "/" (unix::getenv "PWD")))));; path名とfnameが同じと仮定
    )
  fname
  )

(defun calc-xrange (xrange rs-list)
  (let ( (start-idx 0) (stop-idx 0) )
    (if xrange
        (progn
          (until (< (car xrange) (send (elt rs-list start-idx) :time)) (incf start-idx))
          (until (< (cadr xrange) (send (elt rs-list stop-idx) :time)) (incf stop-idx))
          (list start-idx stop-idx) )
      (list 0 (length rs-list)))
    ))

(defun low-pass-filter (target-list &key (dt 0.004) (cutoff-f nil))
  (let ( rate )
    (when cutoff-f
      (setq rate (/ (* 2 pi cutoff-f dt) (+ 1 (* 2 pi cutoff-f dt))))
      (dotimes (i (1- (length target-list)))
        (setf (elt target-list (1+ i)) (+ (* (- 1 rate) (elt target-list i)) (* rate (elt target-list (1+ i)))) )) )
    target-list
    ))

(defun plot-foot-vel
  (&key (rs-list *rs-list*)
        (add-fname nil) (fname "Slide Foot Velocity")
        (cutoff-f nil) (xrange nil)
        (non-parse-flg nil) )
  (let* ( (xrange (calc-xrange xrange rs-list)) (data (subseq rs-list (car xrange) (cadr xrange)))
          (frame-rate 250) (lfoot-act-vel-list nil) (lfoot-ref-vel-list nil) )
    (send *robot* :angle-vector (send (car data) :reference-angle-vector))
    (send *robot* :fix-leg-to-coords (make-coords) :rleg)
    (setq last-lfoot-act-pos (send *robot* :lleg :end-coords :worldpos))

    (send *robot* :angle-vector (send (car data) :angle-vector))
    (send *robot* :fix-leg-to-coords (make-coords) :rleg)
    (setq last-lfoot-ref-pos (send *robot* :lleg :end-coords :worldpos))

    (dolist (x data)
      ;; センサ値代入
      (send *robot* :angle-vector (send x :angle-vector))
      (send *robot* :fix-leg-to-coords (make-coords) :rleg)
      (push-back (* (norm (subseq (v- (send *robot* :lleg :end-coords :worldpos) last-lfoot-act-pos) 0 2)) frame-rate) lfoot-act-vel-list)
      (setq last-lfoot-act-pos (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :worldpos))

      ;; 目標値代入
      (send *robot* :angle-vector (send x :reference-angle-vector))
      (send *robot* :fix-leg-to-coords (make-coords) :rleg)
      (push-back (* (norm (subseq (v- (send *robot* :lleg :end-coords :worldpos) last-lfoot-ref-pos) 0 2)) frame-rate) lfoot-ref-vel-list)
      (setq last-lfoot-ref-pos (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :worldpos))

      )
    (graph-view
     (list (low-pass-filter (if non-parse-flg *lfoot-act-vel-list* lfoot-act-vel-list) :cutoff-f cutoff-f) lfoot-ref-vel-list)
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (list "Actual Slide Foot Velocity" "Reference Slide Foot Velocity")
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph t
     :xlabel "Time[s]"
     :ylabel "Slide Foot Velocity"
     )
    (setq *lfoot-act-vel-list* lfoot-act-vel-list)
    (warn "")
    ))

(defun plot-root-rot
  (&key (axis 0) (rs-list *rs-list*)
        (add-fname nil)   (fname (format nil "Actual Root Rot ~a" (case axis (0 "Yaw") (1 "Pitch") (2 "Roll"))))
        (add-keylist nil) (def-keylist (list "Root Rot Angle"))
        (cutoff-f nil) (xrange nil) (replot nil) (init-zero nil))
  (let* ( (xrange (calc-xrange xrange rs-list)) 
          (data (subseq rs-list (car xrange) (cadr xrange))) )
    (graph-view
     (list (mapcar #'(lambda (x) (rad2deg (elt (car (send (send x :root-coords) :rpy-angle)) axis))) data))
     (mapcar #'(lambda (x) (if init-zero (- (send x :time) (send (car data) :time)) (send x :time))) data)
     :keylist (if add-keylist (mapcar #'(lambda (def-key add-key) (format nil "~a ~a" def-key add-key)) def-keylist add-keylist) def-keylist )
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph t
     :xlabel "Time[s]"
     :ylabel "Angle [Degree]"
     :replot replot
     )
    ))

;; replotできるように拡張
(defun graph-view
  (ordinate-list
   &optional (abscissa-list (user::range (length (car ordinate-list))))
   &key (title "Graph") (xlabel "X") (ylabel "Y") (zlabel "Z")
   (dump-graph nil) (graph-fname (format nil "~A.eps" (substitute #\_ (elt " " 0) title)))
   ;;(mode "points")
   (mode "lines")
   keylist xrange yrange zrange
   x11 additional-func
   no-dump ((:graph-instance gp) (if (boundp 'user::*gp*) user::*gp* (setq user::*gp* (gnuplot))))
   ;; (fname (format nil "data~A" (sys::address gp)))
   (raw-date (unix:localtime))
   (fname (reg-replace* " " "0" (format nil "data_20~2d-~2d-~2d-~2d-~2d-~2d" 
                                        (- (elt raw-date 5) 100) (+ (elt raw-date 4) 1) (elt raw-date 3) (elt raw-date 2) (elt raw-date 1) (elt raw-date 0) )))
   (replot nil))
  (labels ((gen-range-string
            (range)
            (if range (format nil "[~A:~A]" (car range) (cadr range)) "[]"))
           (2d-or-3d (r-2d r-3d) (if (atom (car abscissa-list)) (eval r-2d) (eval r-3d))))
    (unless keylist (setq keylist (user::range (length ordinate-list))))
    ;; dump dat file
    (unless no-dump
      (with-open-file
       (f (format nil "/tmp/~A.dat" fname) :direction :output)
       (format f (2d-or-3d "# x vals..~%" "# x y vals..~%"))
       (dotimes (i (length abscissa-list))
         (if (atom (car abscissa-list))
             (format f "~A " (elt abscissa-list i))
           (format f "~A ~A " (elt (elt abscissa-list i) 0) (elt (elt abscissa-list i) 1)))
         ;;(dolist (d ordinate-list) (format f "~A " (elt d i)))
         (dolist (d ordinate-list)
           (if (< i (length d))
               (format f "~A " (elt d i))))
         (format f "~%")
         )
       )
      )
    
    ;; plot
    (mapcar #'(lambda (d1 d2)
                (send gp :command (format nil "set ~A \"~A\"" d1 d2)))
            '(user::title user::xlabel user::ylabel user::zlabel)
            (list title xlabel ylabel zlabel))
    (if additional-func (funcall additional-func))
    (dotimes (i (length ordinate-list))
      (send gp :command
            (format nil "~A \"/tmp/~A.dat\" using ~A title \"~A\" with ~A"
                    (cond
                     ((and (= i 0) (not replot))
                      (apply #'format
                             (list nil
                                   (2d-or-3d "plot ~A ~A" "splot ~A ~A ~A")
                                   (gen-range-string xrange)
                                   (gen-range-string yrange)
                                   (2d-or-3d nil (gen-range-string zrange)))))
                     (t "replot"))
                    fname
                    (format nil "~A:~A" (2d-or-3d "1" "1:2") (+ i (2d-or-3d 2 3)))
                    (elt keylist i)
                    mode))
      )
    (if x11 (send gp :command "set terminal X11"))
    (when dump-graph
      (unix:usleep 200000)
      (send gp :save graph-fname)
      (unix:usleep 200000)
      )
    ))

;; *log* *rs-list* の作成, fname-with-pathのparse
(defun prepare-plot ( &key (create nil) (fname nil) (robot *robot-class*) (rs-list))
  (let* ( (fname-with-path (parse-fname-with-path fname))
          (fname (parse-fname fname))
          (rs-fname (concatenate string (reg-replace "[A-z,0-9,_,-]*$" "" fname-with-path) "rs-list.l")))

    (when (or (not (boundp '*log*)) create)
      (warning-message 3 "create new *log* from ~a~%" fname-with-path)
      (setq *log* (create-log-interface fname-with-path :robot robot)))
    
    (cond
     ( (and (probe-file rs-fname) (not create) )
       (warning-message 3 "load preserved robot-state list (~a)~%" rs-fname)
       (load rs-fname) )
     ( t
       (warning-message 3 "now converting *log* to robot-state list~%")
       (setq *rs-list* (create-rs-list :ri *log*))
       (unix:system "beep; notify-send \"finised convering log file to robot state list\" -t 0")
       (dump-loadable-structure rs-fname *rs-list*) )
     )

    (if rs-list (setq 'rs-list (copy-object *rs-list*)))
    fname-with-path
    ))
