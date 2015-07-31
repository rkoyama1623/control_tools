;; hrpsysシーケンスファイルから直接、実機またはシミュレータを動かす
;; OpenHRP simulatorの使いかた
;; 8号館602 avocadoにて
;; cd ${CVSDIR}/hrp2/OpenHRP/eus
;; eus test-sim.l
;; (init) or (init :nosim nil);; simulationが起動，あとは実機と同じ
;; (send *ci* :stop-zmp-override)
;; (send *ci* :set-abc-param :check-shuffling nil)
;; (send *ci* :load-pattern "/tmp/****" 5000)
;; (send *ci* :send "seq" ":load ***.pos nil ***.zmp ***.waist")
;; (send *ci* :stop-and-save-log "choreonoid-test-***")


;; (load "~/prog/hrp2/plugins/test-log-parser.l")
(if (unix:getenv "OLD_HRPSYS") (warning-message 1 "!!!!!!!!!!Using Old hrpsys!!!!!!!!!!~%") (warning-message 3 "Using New hrpsys~%"))

(if (not (unix:getenv "OLD_HRPSYS"))
  (let ( (robot-name (unix:getenv "ROBOT")) );; new hrpsys
    (cond
     ( (equal robot-name "HRP2JSK")
       (warning-message 3 "loading robot interface file...~%")
       (require :hrp2jsk-interface    "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsk-interface.l")
       (warning-message 3 "initializing robot interface...~%")
       (hrp2jsk-init)    (setq *robot* *hrp2jsk*)    )
     ( (equal robot-name "HRP2JSKNT")
       (warning-message 3 "loading robot interface file...~%")
       (require :hrp2jsknt-interface  "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
       (warning-message 3 "initializing robot interface...~%")
       (hrp2jsknt-init)  (setq *robot* *hrp2jsknt*)  )
     ( (equal robot-name "HRP2JSKNTS")
       (warning-message 3 "loading robot interface file...~%")
       (require :hrp2jsknts-interface "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
       (warning-message 3 "initializing robot interface...~%")
       (hrp2jsknts-init) (setq *robot* *hrp2jsknts*) )
     )
    )
  (when (not (boundp '*hrp2*)) (setup-model-for-hrp2) (setq *robot* *hrp2*)  );; old hrpsys
  )

;; (load "euslib/jsk/gnuplotlib.l")

(unless (boundp '*usage-funcs*) (setq *usage-funcs* nil))
(push-back #'(lambda () (warn "~%play-hrpsysfile.l~%")) *usage-funcs*)
(push-back #'(lambda () (warning-message 3 "(setup-real-for-hrp2)~%")) *usage-funcs*)
(push-back #'(lambda () (warning-message 3 "(scale-hrpsysfile :fname <FILE NAME>)~%")) *usage-funcs*)
(push-back #'(lambda () (warning-message 3 "(copy-files :fname <FILE NAME> :real t)~%")) *usage-funcs*)
(push-back #'(lambda () (warning-message 3 "(init-pose :fname <FILE NAME> :real t)~%")) *usage-funcs*)
(push-back #'(lambda () (warning-message 3 "(play-hrpsysfile :fname <FILE NAME> :real t)~%")) *usage-funcs*)
(push-back #'(lambda () (warning-message 3 "(save-log :fname <FILE NAME> :add-fname <FILE NAME OPTION>)~%")) *usage-funcs*)
(push-back #'(lambda () (warn "~%")) *usage-funcs*)
(push-back #'(lambda () (warning-message 3 "(set-pgain <l-crotch dgain>)~%")) *usage-funcs*)
(push-back #'(lambda () (warning-message 3 "(set-dgain <l-crotch dgain>)~%")) *usage-funcs*)
(push-back #'(lambda () (warn "~%")) *usage-funcs*)

(load "log-tools.l")

(defun usage()
  (warn "[[usage]]")
  (mapc #'(lambda (func) (funcall func)) *usage-funcs*))
(usage)

;; (if (not (boundp '*ci*)) (setup-real-for-hrp2))

;; (when (not (boundp '*gnuplot*))
;;   (if (boundp '*gplot*) (setq *gnuplot* *gplot*)  (setq *gnuplot* (gnuplot)))
;;   (send *gnuplot* :command "set size 1.5,1")(send *gnuplot* :command "set size 1,1"))

;; 実機かシミュレーションか判定
(defun check-sim ()
  (if (ros::get-param "use_sim_time" nil);; シミュレーションフラグ
      t
    nil)
  )

;; 16・17号機のposファイルを修正する
(defun modify-posfile (path fname)
    (when (or (equal (send *robot* :name) "HRP2JSKNT") (equal (send *robot* :name) "HRP2JSKNTS"))
      (unix:system (concatenate string "cp " path fname ".pos "  path fname "_.pos"))
      (unix:system (concatenate string "cut -d ' ' -f1-35 " path fname "_.pos" "| cat >  " path fname ".pos"))
      )
    )

;; 実際に実機またはローカルの/tmpにファイルをコピー
(defun exec-copy (path fname)
  (if (check-sim) 
      ;; hrpsysシミュレータ用 localの/tmpにコピー
      (unix::system (concatenate string "cp " path fname ".* /tmp/" ) )
    ;; 実機
    (cond ( (equal (send *robot* :name) "HRP2JSK")    (unix::system (concatenate string "scp " path fname ".* grxuser@hrp2007c:/tmp") ) )
          ( (equal (send *robot* :name) "HRP2JSKNT")  (unix::system (concatenate string "scp " path fname ".* grxuser@hrp2016c:/tmp") ) )
          ( (equal (send *robot* :name) "HRP2JSKNTS") (unix::system (concatenate string "scp " path fname ".* grxuser@hrp2017c:/tmp") ) )
          ( t (warning-message 1 "No Setting for The Robot~%") )
          )
    )
  )

;; 関節角速度ファイル作成
;; 自分のPCから実行
(defun copy-files (&key (fname "SampleMotion") (ofname nil) (real nil))
  (let ( motionname
         zmpline velfile optfile posfile zmpfile path
         ;; (extention-list '("pos" "hip" "waist" "zmp" "vel" "optionaldata" "wrenches"))
         (extention-list '("pos" "waist" "zmp" "vel" "optionaldata" "wrenches"))
         )

    (if (not ofname) (setq ofname fname))

    (setq zmp-x-list nil zmp-y-list nil zmp-z-list nil);; zmpリスト

    (setq motionname (car (reg-split "_" fname))
          path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")
          velfile (concatenate string path fname ".vel")
          posfile (concatenate string path fname ".pos")
          zmpfile (concatenate string path fname ".zmp")
          optfile (concatenate string path fname ".optionaldata")
          numjoints (length (send *robot* :angle-vector))
          )

    ;; wrenchファイルの作成 5行目:flz 6行目:frz
    (let ( (sfcfile (concatenate string fname ".SFC_dat"))
           (wrenchfile (concatenate string fname ".wrenches")) )
      (unix:system (concatenate string "sed -e '1,1d' " path sfcfile "| awk '{print $1, 0,0,$5,0,0,0, 0,0,$6,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0}' | cat > " path wrenchfile))
      )

    ;; velfile作成
    (warn "~%now creating vel file and optfile~%")
    (with-open-file
     (vf velfile :direction :output)
     (with-open-file
      (pf posfile :direction :input)
      (with-open-file
       (zf zmpfile :direction :input)
       (let ( (eof (gensym))  posline (zmpeof (gensym)) curzmp prezmp (diffzmp 0) (idx 0) (dt 5) )
         (while (not (eq eof (setq posline (read-line pf nil eof))))
           (setq zmpline (read-line zf nil zmpeof))

           (setq curzmp (coerce (mapcar #'read-from-string (cdr (reg-split " +" zmpline))) float-vector))
           (push-back (elt curzmp 0) zmp-x-list) (push-back (elt curzmp 1) zmp-y-list) (push-back (elt curzmp 2) zmp-z-list);; リストへpush
           (if prezmp (setq diffzmp (* 1000 (/ (norm (v- curzmp prezmp)) dt))))
           (cond
            ;; ギャップが大きい時
            ( (> diffzmp 50) (warning-message 1 " ~a" idx) );; [mm/msec]
            ;; 通常時
            ( t  (setq prezmp curzmp) )
            )

           ;; ;; optional data 書き込み
           ;; (format of "~a" (read-from-string (car (reg-split " +" posline))));; 時刻書き込み
           ;; (format of " ~a ~a ~a ~a ~a ~a ~a ~a ~%" 1 0 0 0   5 5 5 5) ;; 常に右足のみ接触で残りの足は非接触

           ;; 速度ファイル書き込み
           (format vf "~a" (read-from-string (car (reg-split " +" posline))));; 時刻書き込み
           (dotimes (i numjoints) (format vf " ~a" 0.0) )
           (format vf "~%")

           (incf idx)
           )))))

    (warn "~%last index ~a~%" (1- (length zmp-y-list)))
    (send *gnuplot* :draw (subseq zmp-y-list 0 (- (length zmp-y-list) 3) ))

    ;; 最終2行削除
    (mapc #'(lambda (extemtion)
              (unix:system (concatenate string "sed  -e '$d' " path fname "." extemtion "| sed -e '$d' | cat >  " path fname "_." extemtion)) )
              extention-list )

    ;; posファイル 関節角度個数合わせ (HRP2 16・17号機)
    (modify-posfile path (concatenate string fname "_"))

    ;; ファイルコピー
    (warn "~%now copying files~%")
    (exec-copy path (concatenate string fname "_"))

    (unix::system (concatenate string "rm -f " path fname "_.*"));; 一次生成ファイル削除
    (unix::system (concatenate string "rm -f " path fname "__.*"))

    ))

;; simの時はavocadで実行
(defun play-hrpsysfile-old (&key (fname "SampleMotion_200") (real nil) (impedance nil) (tm 5000) (save-log t) )
  (when real
    (when save-log (send *ci* :stop-and-save-log "trash"))
    (send *ci* :start-grasp)
    (send *ci* :stop-zmp-override)
    (send *ci* :set-abc-param :check-shuffling nil)
    (warning-message 3 "Push Enter to start~%")
    (do-until-key)
    (when save-log (send *ci* :start-log))
    (send *ci* :load-pattern (concatenate string "/tmp/" fname) tm)
    (when impedance (send *ci* :start-impedance :lleg))
  ))

(defun prepare-experiment ()
  (if (not (equal (send *ri* :get-st-algorithm) :eefm))
      (send *ri* :set-st-param :st-algorithm :eefm))
  (unless (= (cdr (assoc 'ros::_emergency_check_mode (send (send *ri* :get-st-param) :slots))) 0)
    (send *ri* :set-st-param :emergency-check-mode 0))
  (send *ri* :set-log-maxlength (* 250 45))
  (send *ri* :start-st)
  )

(defun play-hrpsysfile-impl (&key fname (real nil) (tm 10) (save-log t))
  (let (path)
  (warning-message 3 "Push Enter to start~%")
  (prepare-experiment)
  (do-until-key)
  (if save-log (send *ri* :start-log))
  (setq *__fname__* (reg-replace* "_$" "" fname))
  (when real
    (setq path "/tmp/")
    (send *ri* :load-pattern (concatenate string path fname) tm) )
  ))

(defun play-hrpsysfile (&rest args &key fname (real nil) (save-log t) (tm 10) &allow-other-keys)
  (when (boundp '*ci*) (play-hrpsysfile-old  :fname fname :real real :save-log save-log))
  (when (boundp '*ri*) (play-hrpsysfile-impl :fname fname :real real :save-log save-log :tm tm))
  )

(defun play-demo ( &key (music-file "/home/jskuser/love-cir_demo_x82.wmv") )
  (when (not (boundp '*sh*))
    (setq *sh* (piped-fork "sh"))
    (format *sh* "vlc -I rc --no-loop --no-repeat --volume 450~%") (unix:sleep 1) )
  (format *sh* "clear~%")
  (format *sh* "add ~A~%" (reg-replace* "/" "\/" music-file))
  (unix:usleep 10000)
  (format *sh* "pause~%")

  (play-hrpsysfile :fname "love-cir-slow-moderate-side-point_" :real t :save-log nil)
  (unix::usleep (round (* 1000 1000 9.6)))

  ;; (format *sh* "add ~A~%" (reg-replace* "/" "\/" music-file))
  (format *sh* "play~%")
  ;; (if music-file
  ;;     (unix::system (concatenate string "vlc --no-loop --no-repeat --volume 450 -f --play-and-exit " music-file))
  ;;   (unix::system "vlc --no-loop --no-repeat --volume 450 -f --play-and-exit ~/Dropbox/Videos/research（研究）/love-cir/love-cir_demo_x82.wmv"))

  )

;; hrpsysファイルのタイムスケールを変更する (足りない場合は線形補間)
;; 現状100fpsから200fpsの変更のみ可能
(defun scale-hrpsysfile (&key (fname "SampleMotion"))
  (let (motionname hipfile posfile waistfile zmpfile path )

    (setq motionname fname
          path (concatenate string "/home/" (unix::getenv "USER") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/")
          hipfile (concatenate string path fname "_100fps.hip")
          posfile (concatenate string path fname "_100fps.pos")
          waistfile (concatenate string path fname "_100fps.waist")
          zmpfile (concatenate string path fname "_100fps.zmp")

          hipfile_ (concatenate string path fname ".hip")
          posfile_ (concatenate string path fname ".pos")
          waistfile_ (concatenate string path fname ".waist")
          zmpfile_ (concatenate string path fname ".zmp")

          numjoints (length (send *robot* :angle-vector))
          )

    ;; 各種ファイルをスケール
    (warn "~%now scaling posfile~%")
    (mapc 
     #'(lambda (infile outfile)
         (with-open-file
          (inf infile :direction :input)
          (with-open-file
           (outf outfile :direction :output)
           (let ( (eof (gensym))  curline prevec curvec midvec )
             (while (not (eq eof (setq curline (read-line inf nil eof))))
               (setq curvec (coerce (mapcar #'read-from-string (reg-split " +" curline)) float-vector))

               ;; 補間時刻データ書き込み
               (when prevec
                 (setq midpos (scale 0.5 (v+ prevec curvec)))
                 (dotimes (i (length midpos)) (format outf "~a " (elt midpos i)))
                 (format outf "~%")
                 )

               ;; 非補間時刻データ書き込み
               (format outf "~a~%" curline)

               (setq prevec curvec)
               )))))
     (list hipfile posfile waistfile zmpfile) (list hipfile_ posfile_ waistfile_ zmpfile_)
     )

    ))

;; モーションの初期姿勢へ遷移 (hrpsysシーケンスファイルの1行目のみを実行)
(defun init-pose (&key (fname "SampleMotion") (tm 2000) (real nil))
  (let* ( (motionname (car (reg-split "_"  fname)))
          (path (concatenate string (unix::getenv "HOME") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/"))
          )

    (mapc #'(lambda (extemtion)
              (unix:system (concatenate string "head -n 1 " path fname "." extemtion "| cat >  " path fname "_init." extemtion)) )
          ;; (list "hip" "waist" "zmp" "vel" "pos" "optionaldata" "wrenches")
          (list "waist" "zmp" "vel" "pos" "optionaldata" "wrenches") )

    ;; posファイル 関節角度個数合わせ (HRP2 16・17号機)
    (modify-posfile path (concatenate string fname "_init"))

    ;; ファイルをコピー
    (exec-copy path (concatenate string fname "_init"))

    (when real
      (cond
       ( (boundp '*ci*)
         (send *ci* :start-grasp)
         (send *ci* :stop-zmp-override)
         (send *ci* :set-abc-param :check-shuffling nil)

         (send *ci* :set-impedance-param :moment-gain 0 :legs)
         (send *ci* :set-impedance-param :axis-p :z :legs)
         (send *ci* :set-impedance-param :mdk #f(0.01 5 3) :legs)
         (send *ci* :set-ref-force #f(0 0 200) 1000 :legs)

         (send *ci* :load-pattern (concatenate string "/tmp/" fname "_init") tm) )
       ( (boundp '*ri*)
         (send *ri* :load-pattern (concatenate string "/tmp/" fname "_init")) )
       )
      )

    (unix::system (concatenate string "rm -f " path fname "_init.*"));; 一次生成ファイル削除
    (unix::system (concatenate string "rm -f " path fname "_init_.*"));; 一次生成ファイル削除

    ))

(defun reset-pose ()
  (let* ( (fname-with-path "/tmp/reset-pose")
          ;; (waist-coord)
          (posfile      (format nil "~a.pos"          fname-with-path)) pf
          (waistfile    (format nil "~a.waist"        fname-with-path)) waf
          (zmpfile      (format nil "~a.zmp"          fname-with-path)) zf
          (wrenchfile   (format nil "~a.wrenches"     fname-with-path)) wf
          (optionalfile (format nil "~a.optionaldata" fname-with-path)) of )

    (send *robot* :fix-leg-to-coords (make-coords))
    (setq waist-coord (reduce #'(lambda (coord0 coord1) (make-coords :pos (scale 0.5 (v+ (send coord0 :worldpos) (send coord1 :worldpos)))))
                             (send-all (send-all (send *robot* :legs :crotch-r) :child-link) :worldcoords)))

    ;; waistファイル作成
    (with-open-file
     (waf waistfile :direction :output)
     (format waf "0 ")
     (mapc #'(lambda (val) (format waf "~f " (/ val 1000))) (coerce (send waist-coord :worldpos) cons))
     (mapc #'(lambda (val) (format waf "~f " val)) (car (send waist-coord :rpy-angle)))
     )
    
    ;; zmpファイル作成
    (with-open-file
     (zf zmpfile :direction :output)
     (format zf "0 ")
     (mapc #'(lambda (val)
               (format zf "~f " (/ val 1000)))
           (coerce (send waist-coord :inverse-transform-vector #f(0 0 0)) cons))
     )

    ;; posfile作成  
    (with-open-file
     (pf posfile :direction :output)
     (format pf "0 ")
     (mapc #'(lambda (angle) (format pf "~f " (deg2rad angle))) (coerce (send *robot* :reset-pose) cons))
     (format pf "~%")
     )

    ;; wrenchfile作成
    (with-open-file
     (wf wrenchfile :direction :output)
     (format wf "0  0 0 ~f 0 0 0  0 0 ~f 0 0 0  0 0 0 0 0 0  0 0 0 0 0 0~%" (* 9.8 1e-3 (* 0.5 (send *robot* :weight))) (* 9.8 1e-3 (* 0.5 (send *robot* :weight))))
     )

    ;; optionalfile作成
    (with-open-file
     (of optionalfile :direction :output)
     (format of "0.0 1 1 0 0 5 5 5 5~%")
     )

    (exec-copy "/tmp/" "reset-pose")

    (send *ri* :load-pattern "/tmp/reset-pose")

    ))

(defun set-gain ( &optional (gain-ratio 1.0) )
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (setf (elt vec (send *hrp2* :lleg :crotch-r :servo :no)) gain-ratio)
  (send *ci* :gain-vector vec 1000 t nil)
  )
(defun set-pgain ( &optional (gain-ratio 1.0) )
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (setf (elt vec (send *hrp2* :lleg :crotch-r :servo :no)) gain-ratio)
  (send *ci* :gain-vector vec 1000 t nil)
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (send *ci* :dgain-vector vec 1000)
  )
(defun set-dgain ( &optional (gain-ratio 1.0) )
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (setf (elt vec (send *hrp2* :lleg :crotch-r :servo :no)) gain-ratio)
  (send *ci* :gain-vector vec 1000 t nil)
  (setq vec (fill (instantiate float-vector (send *hrp2* :dof)) 1.0))
  (send *ci* :pgain-vector vec 1000 t)
  )


(defun shuffle-walk-old ( &optional (term 1) )
  (send *ci* :stop-st)
  (send *ci* :start-grasp)
  (send *ci* :stop-zmp-override)
  (send *ci* :set-abc-param :check-shuffling nil)
  (init-pose :fname "2dof-slide_short_1smu_1dmu_2D" :real t :tm 5000)
  ;; (init-pose :fname "2dof-slide_left-high-waist_1smu_1dmu_1e-7w_2D" :real t)
  (unix:sleep 2)
  (dotimes (i term)
    (send *ci* :load-pattern "/tmp/2dof-slide_short_1smu_1dmu_2D_" 2000)
    ;; (send *ci* :load-pattern "/tmp/2dof-slide_left-high-waist_1smu_1dmu_1e-7w_2D_" 2000)
    (unix:usleep (round (* 1e6 (+ 3 1.6))))
    (send *ci* :load-pattern "/tmp/2dof-slide_right_1smu_3dmu_2D_" 2000)
    ;; (send *ci* :load-pattern "/tmp/2dof-slide_right-high-waist_1smu_1dmu_1e-7w_2D_" 2000)
    (unix:usleep (round (* 1e6 (+ 3 1.6))))
    )
  )

(defun shuffle-side-walk-old ( &optional (term 1) )
  (send *ri* :start-st)
  (init-pose :fname "2dof-slide-left_short_open_58-5mass_0-5smu_0-5dmu_1w_2D" :real t :tm 5)
  ;; (init-pose :fname "2dof-slide_left-high-waist_1smu_1dmu_1e-7w_2D" :real t)
  (unix:sleep 5)
  (dotimes (i term)
    (send *ri* :load-pattern "/tmp/2dof-slide-left_short_open_58-5mass_0-5smu_0-5dmu_1w_2D_" 1)
    (unix:usleep (round (* 1e6 (+ 1 1.6))))
    (send *ri* :load-pattern "/tmp/2dof-slide-left_short_close_58-5mass_1smu_1-5dmu_1w_2D_" 1.5)
    (unix:usleep (round (* 1e6 (+ 1.5 1.6))))
    )
  )

(defun shuffle-side-walk ( &optional (term 1) )
  (setq *__fname__* (format t "shuffle-side-walk-~s" term))
  ;; (copy-files :fname "2dof-slide-left_short_comb_1smu_1dmu_1w_2D" :real t)
  (prepare-experiment)
  (init-pose :fname "2dof-slide-left_short_comb_1smu_1dmu_1w_2D" :real t :tm 3)
  ;; (init-pose :fname "2dof-slide_left-high-waist_1smu_1dmu_1e-7w_2D" :real t)
  (unix:sleep 3)
  (dotimes (i term)
    (send *ri* :load-pattern "/tmp/2dof-slide-left_short_comb_1smu_1dmu_1w_2D_" 1)
    (unix:usleep (round (* 1e6 (+ 1 2.5))))
    )
  )

(defun shuffle-forward-walk ( &optional (term 1) &key (tm 1) )
  (setq *__fname__* (format t "shuffle-forward-walk-~s" term))
  (prepare-experiment)
  (init-pose :fname "3dof-slide-forward_short_continue_1smu_1-1dmu_1w_2D" :real t :tm 3)
  (unix:sleep 3)
  (dotimes (i term)
    (send *ri* :load-pattern "/tmp/3dof-slide-forward_short_continue_1smu_1-1dmu_1w_2D_" 1)
    (unix:usleep (round (* 1e6 (+ tm 2.5))))
    )
  )