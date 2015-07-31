(load "~/prog/hrp2/plugins/test-log-parser.l")
(load "package://euslib/irteus_proposals/motion-lib-proposal.l")

(setup-model-for-hrp2) (setq *robot* *hrp2*)

(defun create-log-interface (fname-with-path &allow-other-keys)
  (setup-log-parser (format nil "~a-astate.log" fname-with-path)) )

(defun create-rs-list (&key (ri *log*))
  (convert-to-robot-state-data-from-ri :ri *log*) )

;; 16号機,17号機のみ
(defun inverse-hrp-angle-vector (angles)
  (mapc #'(lambda (idx) (setf (elt angles idx) (- (elt angles idx)))) '(1 5 14 16 19 20 22 23 25))
  angles
  )

(defun convert-to-robot-state-data-from-ri
  (&key (ri *ri*))
  (let ((rs-list) (init-tm) (state))
    (send ri :index 1) (send *log* :proc)
    (do-until-key-with-check
     (progn (setq state (send ri :state)) (send *log* :proc))
     (progn
       (unless init-tm
         (setq init-tm (send state :time)))
       (push
        (instance robot-state-data :init
                  (send state :potentio-vector)
                  ;; (send state :worldcoords)
                  (send state :worldcoords-from-rpy-vector *hrp2* 0)
                  (+ (- (elt (send state :time) 0) (elt init-tm 0))
                     (/ (- (elt (send state :time) 1) (elt init-tm 1)) 1000000.0))
                  :torque-vector (send state :torque-vector)
                  :reference-root-coords (send state :reference-worldcoords)
                  :reference-angle-vector (send state :reference-vector)
                  :wrenches (mapcar #'(lambda (i) (coerce (append (coerce (subseq (send (send *log* :state) :force-vector)  i (+ i 3)) cons)
                                                                  (coerce (subseq (send (send *log* :state) :moment-vector) i (+ i 3)) cons)) float-vector))
                                    '(0 3 6 9)) )
        rs-list)
       ))
    (reverse rs-list)))

;; 関節角速度リスト作成
(defun make-list-from-input (&key (fname "SampleMotion_50") (span 10))
  (let* ( (motionname (string-trim  "_0123456789" fname))
          (path (concatenate string (unix::getenv "HOME") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" motionname "/") )
          (posfile (concatenate string path fname ".pos"))
          (numjoints (length (send *robot* :lleg :angle-vector)))
          vellist )
    (setq l-crotch-r-ref-vel (list 0) l-crotch-p-ref-vel (list 0) l-crotch-y-ref-vel (list 0)
          l-knee-p-ref-vel (list 0) l-ankle-r-ref-vel (list 0) l-ankle-p-ref-vel (list 0)
          r-crotch-r-ref-vel (list 0) r-crotch-p-ref-vel (list 0) r-crotch-y-ref-vel (list 0) 
          r-knee-p-ref-vel (list 0) r-ankle-r-ref-vel (list 0) r-ankle-p-ref-vel (list 0)
          )

    (with-open-file ;; hrpsysファイル読み込み
     (pf posfile :direction :input)
     (let ( (eof (gensym)) posline poslist (prev-poslist nil) (idx 0) )
       (warn "now creating list~%")
       (while (not (eq eof (setq posline (read-line pf nil eof))))
         (when (= (mod idx span) 0)
           (setq poslist (mapcar #'(lambda (val) (read-from-string val)) (cdr (reg-split " +" posline))))

           (when prev-poslist
             (setq vellist (mapcar #'(lambda (pos prev-pos) (* (/ 200.0 span) (- pos prev-pos))) poslist prev-poslist));; rad/sec
             (mapc #'(lambda (limb angle-lists)
                       (mapc #'(lambda (joint angle-list) (push-back (elt vellist (cadr (send *robot* limb joint :info))) angle-list))
                             (list :crotch-r :crotch-p :crotch-y :knee-p :ankle-r :ankle-p) angle-lists) )
                   (list :lleg :rleg)
                   (list (list l-crotch-r-ref-vel l-crotch-p-ref-vel l-crotch-y-ref-vel l-knee-p-ref-vel l-ankle-r-ref-vel l-ankle-p-ref-vel)
                         (list r-crotch-r-ref-vel r-crotch-p-ref-vel r-crotch-y-ref-vel r-knee-p-ref-vel r-ankle-r-ref-vel r-ankle-p-ref-vel) ) )
             )

           (setq prev-poslist (copy-seq poslist))
           )
         (incf idx)
         );; end while
       ));; end with-open-file

    (setq crotch-r-max (make-list (length l-crotch-r-ref-vel) :initial-element (send *robot* :lleg :crotch-r :max-joint-velocity))
          crotch-p-max (make-list (length l-crotch-r-ref-vel) :initial-element (send *robot* :lleg :crotch-p :max-joint-velocity))
          crotch-y-max (make-list (length l-crotch-r-ref-vel) :initial-element (send *robot* :lleg :crotch-y :max-joint-velocity))
          knee-p-max   (make-list (length l-crotch-r-ref-vel) :initial-element (send *robot* :lleg :knee-p :max-joint-velocity))
          ankle-r-max  (make-list (length l-crotch-r-ref-vel) :initial-element (send *robot* :lleg :ankle-r :max-joint-velocity))
          ankle-p-max  (make-list (length l-crotch-r-ref-vel) :initial-element (send *robot* :lleg :ankle-p :max-joint-velocity)) )

    (send *gnuplot* :draw l-crotch-r-ref-vel l-crotch-p-ref-vel l-crotch-y-ref-vel l-knee-p-ref-vel l-ankle-r-ref-vel l-ankle-p-ref-vel)
    ))

;; vecからjointのidxを調べて各リストにpush
(defun push-each-list (model target-vec link-name-list out-list)
  (cond
   ( (= (send (elt (send model :links) 5) :joint :joint-dof) 3);; kinect-sdk-bvhモデル
     (mapc #'(lambda (output-list link-idx)
               (push-back (elt target-vec link-idx) output-list))
           out-list link-name-list
           )
     )
   ( t;; 軸関節
     (mapc #'(lambda (output-list link-name)
               (push-back (elt target-vec (cadr (send (send (find link-name (send model :links) :test #'(lambda (val1 val2) (equal val1 (send val2 :name))))
                                                            :joint) :info))) output-list))
           out-list link-name-list
           )))
  )

;; 実機実行時のログからリストと6軸センサ出力ファイルを作成
;; span=1 にしないと 作成されたファイルはbvhデータと同期しない
(defun make-list-and-file-from-log (&rest args
                                          &key (start 0) stop (span 10) start-tm stop-tm tm-span idx-range (tm-range 2) (offset-tm 0)
                                          (wait nil) (fix-leg nil) (proc-length nil) (plot nil) (return-first t)
                                          (xtics 0.5);; gnuplot目盛り
                                          target-list additional-data
                                          &allow-other-keys)
  (let (av (prev-av nil) tqs zmp ref-zmp waistpos ref-waistpos waistrpy ref-waistrpy free-leg-pos ref-free-leg-pos support-leg free-leg (list-length 0)
           (link-name-list '(:rleg-link0 :rleg-link1 :rleg-link2 :rleg-link3 :rleg-link4 :rleg-link5 :rleg-link6
                                         :lleg-link0 :lleg-link1 :lleg-link2 :lleg-link3 :lleg-link4 :lleg-link5 :lleg-link6
                                         :torso-link0 :torso-link1 :head-link0 :head-link1
                                         :rarm-link0 :rarm-link1 :rarm-link2 :rarm-link3 :rarm-link4 :rarm-link5 :rarm-link6
                                         :larm-link0 :larm-link1 :larm-link2 :larm-link3 :larm-link4 :larm-link5 :larm-link6)) )
    (setq neck-p-vel (list 0) neck-y-vel (list 0) waist-p-vel (list 0) waist-y-vel (list 0)
          l-crotch-r-vel (list 0) l-crotch-p-vel (list 0) l-crotch-y-vel (list 0)
          l-knee-p-vel (list 0) l-ankle-r-vel (list 0) l-ankle-p-vel (list 0) l-toe-p-vel (list 0)
          r-crotch-r-vel (list 0) r-crotch-p-vel (list 0) r-crotch-y-vel (list 0)
          r-knee-p-vel (list 0) r-ankle-r-vel (list 0) r-ankle-p-vel (list 0) r-toe-p-vel (list 0)
          l-shoulder-r-vel (list 0) l-shoulder-p-vel (list 0) l-shoulder-y-vel (list 0)
          l-elbow-p-vel (list 0) l-wrist-r-vel (list 0) l-wrist-p-vel (list 0) l-wrist-y-vel (list 0)
          r-shoulder-r-vel (list 0) r-shoulder-p-vel (list 0) r-shoulder-y-vel (list 0)
          r-elbow-p-vel (list 0) r-wrist-r-vel (list 0) r-wrist-p-vel (list 0) r-wrist-y-vel (list 0)

          neck-p-tq (list 0) neck-y-tq (list 0) waist-p-tq (list 0) waist-y-tq (list 0)                               
          l-crotch-r-tq (list 0) l-crotch-p-tq (list 0) l-crotch-y-tq (list 0)                     
          l-knee-p-tq (list 0) l-ankle-r-tq (list 0) l-ankle-p-tq (list 0) l-toe-p-tq (list 0)        
          r-crotch-r-tq (list 0) r-crotch-p-tq (list 0) r-crotch-y-tq (list 0)                     
          r-knee-p-tq (list 0) r-ankle-r-tq (list 0) r-ankle-p-tq (list 0) r-toe-p-tq (list 0)        
          l-shoulder-r-tq (list 0) l-shoulder-p-tq (list 0) l-shoulder-y-tq (list 0)               
          l-elbow-p-tq (list 0) l-wrist-r-tq (list 0) l-wrist-p-tq (list 0) l-wrist-y-tq (list 0) 
          r-shoulder-r-tq (list 0) r-shoulder-p-tq (list 0) r-shoulder-y-tq (list 0)               
          r-elbow-p-tq (list 0) r-wrist-r-tq (list 0) r-wrist-p-tq (list 0) r-wrist-y-tq (list 0)

          zmp-y-list (list 0)  ref-zmp-y-list (list 0)
          waist-y-list nil  ref-waist-y-list nil  waist-z-list nil  ref-waist-z-list nil
          waist-roll-list nil  ref-waist-roll-list nil
          waist-pitch-list nil
          waist-yaw-list nil 
          waist-yaw-joint-list nil ref-waist-yaw-joint-list nil
          waist-tilt-list nil
          free-leg-y-list (list 0) ref-free-leg-y-list (list 0) free-leg-z-list (list 0) ref-free-leg-z-list (list 0)
          l-myu-list nil  r-myu-list nil 
          l-fz-list nil r-fz-list nil l-fy-list nil r-fy-list nil
          lfoot-y-list nil rfoot-y-list nil ref-lfoot-y-list nil ref-rfoot-y-list nil
          lfoot-vel-list nil rfoot-vel-list nil ref-lfoot-vel-list nil ref-rfoot-vel-list nil
          last-lfoot-pos nil last-rfoot-pos nil ref-last-lfoot-pos nil ref-last-rfoot-pos nil)

    
    (unless start (setq start 0)) ;; 時刻・logインデクス処理 (start-tmとかstop-tmとか要らない??)
    (if start-tm  (setq start (round (/ start-tm (send *log* :dt)))))
    (if stop-tm   (setq stop  (round (/ stop-tm  (send *log* :dt)))))
    (if tm-span   (setq span  (round (/ tm-span  (send *log* :dt)))) (setq tm-span (* span (send *log* :dt))))
    (if idx-range (setq tm-range (* idx-range tm-span)) (setq idx-range (round (/ tm-range tm-span))))
    (send *log* :index start)

    (if xtics 
        (send *gnuplot* :command (format nil "set xtics ~a" xtics))
      (send *gnuplot* :command "unset xtics"))

    (with-open-file
     (fp (concatenate string (unix::getwd) "/force-and-moment-log.txt") :direction :output :if-does-not-exist :create)
     (block :log-play
       (loop
        (if (or (not (send *log* :proc)) (select-stream (list *standard-input*) 1.0e-8) (and stop (> (send *log* :index) stop)) )
            (return-from :log-play nil))
        
        (if (= (mod (round (/ (- (send *log* :index) start) (float span))) 5) 0)
            (format t " ~A" (send *log* :index)))
        (setq cur-tm (* (send *log* :index) (send *log* :dt)))
        (incf list-length)

        (setq av (send *log* :state :potentio-vector)
              zmp (send *log* :state :zmp-vector)  ref-zmp (send *log* :state :refzmp-vector)
              tqs (send *log* :state :torque-vector)
              ;; waistpos (send *log* :state :waistpos)    waistrpy (send *log* :state :waistrpy)  
              ;; ref-waistpos (send *log* :state :reference-waistpos) ref-waistrpy (send *log* :state :reference-waistrpy)
              )
        (push-back (elt zmp 1) zmp-y-list) (push-back (elt ref-zmp 1) ref-zmp-y-list)

        ;; 支持脚->腰位置   腰->遊脚
        (cond ( (< (elt (send *hrp2* :lleg :end-coords :worldpos) 2) (elt (send *hrp2* :rleg :end-coords :worldpos) 2))
                (setq support-leg :lleg  free-leg :rleg) )
              ( t (setq support-leg :rleg free-leg :lleg) ) )

        (when support-leg ;; 実際の軌道
          (setq waistpos (send (send *hrp2* :worldcoords) :inverse-rotate-vector (v- (send *hrp2* :worldpos) (send *hrp2* support-leg :end-coords :worldpos)) )
                free-leg-pos (send (send *hrp2* :worldcoords) :inverse-rotate-vector (v- (send *hrp2* free-leg :end-coords :worldpos) (send *hrp2* :worldpos)) )
                waistrpy (car (send *hrp2* :rpy-angle)) ) )

        ;; 関節速度計算,各リストに出力
        (when prev-av (setq velvec (scale (/ 1 (send *log* :dt) span) (vector-deg2rad (v- av prev-av))) );; rad/sec
              (push-each-list *robot* velvec link-name-list
                              (list r-crotch-y-vel r-crotch-r-vel r-crotch-p-vel r-knee-p-vel r-ankle-p-vel r-ankle-r-vel r-toe-p-vel
                                    l-crotch-y-vel l-crotch-r-vel l-crotch-p-vel l-knee-p-vel l-ankle-p-vel l-ankle-r-vel l-toe-p-vel
                                    waist-y-vel waist-p-vel neck-y-vel neck-p-vel
                                    r-shoulder-p-vel r-shoulder-r-vel r-shoulder-y-vel r-elbow-p-vel r-wrist-y-vel r-wrist-r-vel r-wrist-p-vel 
                                    l-shoulder-p-vel l-shoulder-r-vel l-shoulder-y-vel l-elbow-p-vel l-wrist-y-vel l-wrist-r-vel l-wrist-p-vel )) )
        ;; 各リストにトルク出力
        (push-each-list *robot* tqs link-name-list
                        (list r-crotch-y-tq r-crotch-r-tq r-crotch-p-tq r-knee-p-tq r-ankle-p-tq r-ankle-r-tq r-toe-p-tq
                              l-crotch-y-tq l-crotch-r-tq l-crotch-p-tq l-knee-p-tq l-ankle-p-tq l-ankle-r-tq l-toe-p-tq
                              waist-y-tq waist-p-tq neck-y-tq neck-p-tq
                              r-shoulder-p-tq r-shoulder-r-tq r-shoulder-y-tq r-elbow-p-tq r-wrist-y-tq r-wrist-r-tq r-wrist-p-tq 
                              l-shoulder-p-tq l-shoulder-r-tq l-shoulder-y-tq l-elbow-p-tq l-wrist-y-tq l-wrist-r-tq l-wrist-p-tq ))

        ;; センサ値(実機出力)リスト記録
        (push-back (elt waistpos 1) waist-y-list) (push-back (elt waistpos 2) waist-z-list)
        (push-back (rad2deg (elt waistrpy 2)) waist-roll-list) 
        (push-back (rad2deg (elt waistrpy 1)) waist-pitch-list)
        (push-back (rad2deg (elt waistrpy 0)) waist-yaw-list)
        (push-back (send *hrp2* :torso :waist-y :joint-angle) waist-yaw-joint-list)
        (push-back (elt free-leg-pos 1) free-leg-y-list) (push-back (elt free-leg-pos 2) free-leg-z-list)

        ;; rpyとは別に鉛直軸からの傾きを計算
        (push-back (rad2deg (acos (v. (send  (make-coords :rpy (reverse (send *log* :state :rpy-vector)))  :transform-vector #f(0 0 1))
                                      #f(0 0 1))))
                   waist-tilt-list)

        ;; 足先速度計算
        (push-back (elt (send *hrp2* :lleg :end-coords :worldpos) 1) lfoot-y-list)
        (push-back (elt (send *hrp2* :rleg :end-coords :worldpos) 1) rfoot-y-list)
        (when (and last-lfoot-pos last-rfoot-pos)
          ;; (print (v- (send *hrp2* :lleg :end-coords :worldpos) last-lfoot-pos))
          ;; 右足に対する水平相対速度(ルート座標系)
          ;; (push-back (* (norm (subseq (v- (v- (send *hrp2* :lleg :end-coords :worldpos) (send *hrp2* :rleg :end-coords :worldpos))
          ;;                                 (v- last-lfoot-pos last-rfoot-pos)) 0 2)) 200) lfoot-vel-list)
          ;; 右足に対する水平相対速度(足先座標系)
          (push-back (* (norm (subseq (v- (send *hrp2* :rleg :end-coords :inverse-transform-vector (send *hrp2* :lleg :end-coords :worldpos))
                                          (send *hrp2* :rleg :end-coords :inverse-transform-vector last-lfoot-pos)) 0 2)) 200) lfoot-vel-list)
          (push-back (* (norm (subseq (v- (send *hrp2* :rleg :end-coords :worldpos) last-rfoot-pos) 0 2)) 200) rfoot-vel-list)
          )
        (setq last-lfoot-pos (copy-seq (send *hrp2* :lleg :end-coords :worldpos))  last-rfoot-pos (copy-seq (send *hrp2* :rleg :end-coords :worldpos)))



        ;; 目標軌道(hrp2モデルを目標関節角度に上書きする)
        (send *hrp2* :angle-vector (send *log* :state :reference-vector))
        (send *hrp2* :fix-leg-to-coords (make-coords) '(:rleg))
        (push-back (elt (send *hrp2* :lleg :end-coords :worldpos) 1) ref-lfoot-y-list)
        (push-back (elt (send *hrp2* :rleg :end-coords :worldpos) 1) ref-rfoot-y-list)
        (setq ref-waistpos (send (send *hrp2* :worldcoords) :inverse-rotate-vector (v- (send *hrp2* :worldpos) (send *hrp2* support-leg :end-coords :worldpos)) )
              ref-free-leg-pos (send (send *hrp2* :worldcoords) :inverse-rotate-vector (v- (send *hrp2* free-leg :end-coords :worldpos) (send *hrp2* :worldpos)) ) )

        ;; 目標値リスト作成
        (push-back (elt ref-waistpos 1) ref-waist-y-list)
        (push-back (elt ref-waistpos 2) ref-waist-z-list)
        (push-back 0 ref-waist-roll-list)
        (push-back (send *hrp2* :torso :waist-y :joint-angle) ref-waist-yaw-joint-list)
        (push-back (elt ref-free-leg-pos 1) ref-free-leg-y-list)
        (push-back (elt ref-free-leg-pos 2) ref-free-leg-z-list)

        ;; 足先速度計算
        (when (and ref-last-lfoot-pos ref-last-rfoot-pos)
          ;; (print (v- (send *hrp2* :lleg :end-coords :worldpos) last-lfoot-pos))
          ;; (push-back (* (norm (subseq (v- (v- (send *hrp2* :lleg :end-coords :worldpos) (send *hrp2* :rleg :end-coords :worldpos))
          ;;                                 (v- last-lfoot-pos last-rfoot-pos)) 0 2)) 200) lfoot-vel-list);; 右足に対する相対速度
          (push-back (* (norm (subseq (v- (send *hrp2* :rleg :end-coords :inverse-transform-vector (send *hrp2* :lleg :end-coords :worldpos))
                                          (send *hrp2* :rleg :end-coords :inverse-transform-vector ref-last-lfoot-pos)) 0 2)) 200) ref-lfoot-vel-list);; 右足に対する相対速度
          (push-back (* (norm (subseq (v- (send *hrp2* :rleg :end-coords :worldpos) ref-last-rfoot-pos) 0 2)) 200) ref-rfoot-vel-list)
          )
        (setq ref-last-lfoot-pos (copy-seq (send *hrp2* :lleg :end-coords :worldpos))  ref-last-rfoot-pos (copy-seq (send *hrp2* :rleg :end-coords :worldpos)))

        ;; 足先6軸センサ値記録
        (let (force-vector moment-vector)
          (setq force-vector (send *log* :state :filtered-absolute-force-vector);; 右足・左足の順
                moment-vector  (send *log* :state :filtered-absolute-moment-vector) )
          ;; 摩擦係数リスト記録
          (setq l-force-vec (subseq force-vector 3 6)  r-force-vec (subseq force-vector 0 3) )
          (push-back (/ (norm (subseq l-force-vec 0 2)) (elt l-force-vec 2)) l-myu-list)
          (push-back (/ (norm (subseq r-force-vec 0 2)) (elt r-force-vec 2)) r-myu-list)
          (push-back (elt l-force-vec 1) l-fy-list) (push-back (elt l-force-vec 2) l-fz-list)
          (push-back (elt r-force-vec 1) r-fy-list) (push-back (elt r-force-vec 2) r-fz-list)

          ;; bvhと同じフレームレートで記述
          (when (not (= (round (/ (send *log* :index) (/ 200 30.0))) (round (/ (1- (send *log* :index)) (/ 200 30.0)))))
            (format fp "~a ~a~%" (elt force-vector 5) (elt force-vector 2)) )
          )

        ;; (when plot (send *gnuplot* :draw (eval (elt joint-vel-tq-list 1)) (eval (elt joint-vel-tq-list 2))
        ;;                  :xoffset (* start (send *log* :dt)) :xscale (* 0.005 span)
        ;;                  :xrange (list (+ cur-tm (- offset-tm) (- tm-range)) (+ cur-tm (- offset-tm)))
        ;;                  :title (list (elt joint-vel-tq-list 1) (elt joint-vel-tq-list 2))))
        (when plot (send *gnuplot* :draw (subseq (eval (elt joint-vel-tq-list 2)) (- list-length idx-range))
                         :xoffset (- cur-tm (min (* list-length tm-span) tm-range))
                         :xscale tm-span
                         :xrange (list (+ cur-tm (- offset-tm) (- tm-range)) (+ cur-tm (- offset-tm)))
                         :title (list (elt joint-vel-tq-list 2))))

        (setq prev-av (copy-seq av))
        (view-one :fix-leg fix-leg :ins *log*)
        (if (or target-list additional-data) (apply #'plot-one target-list additional-data args))
        (if wait (unix:usleep (round (* 1000 1000 0.1))))
        (send *log* :incf span)
        (if (and stop (> (send *log* :index) stop)) (return-from make-list-and-file-from-log t))
        ));; end block while
     );; end with-open-file

    ))


(setq joint-vel-tq-list 
      '(r-crotch-y-vel r-crotch-r-vel r-crotch-p-vel r-knee-p-vel r-ankle-p-vel r-ankle-r-vel r-toe-p-vel
                       l-crotch-y-vel l-crotch-r-vel l-crotch-p-vel l-knee-p-vel l-ankle-p-vel l-ankle-r-vel l-toe-p-vel
                       waist-y-vel waist-p-vel neck-y-vel neck-p-vel
                       r-shoulder-p-vel r-shoulder-r-vel r-shoulder-y-vel r-elbow-p-vel r-wrist-y-vel r-wrist-r-vel r-wrist-p-vel
                       l-shoulder-p-vel l-shoulder-r-vel l-shoulder-y-vel l-elbow-p-vel l-wrist-y-vel l-wrist-r-vel l-wrist-p-vel    
                       r-crotch-y-tq r-crotch-r-tq r-crotch-p-tq r-knee-p-tq r-ankle-p-tq r-ankle-r-tq r-toe-p-tq
                       l-crotch-y-tq l-crotch-r-tq l-crotch-p-tq l-knee-p-tq l-ankle-p-tq l-ankle-r-tq l-toe-p-tq
                       waist-y-tq waist-p-tq neck-y-tq neck-p-tq 
                       r-shoulder-p-tq r-shoulder-r-tq r-shoulder-y-tq r-elbow-p-tq r-wrist-y-tq r-wrist-r-tq r-wrist-p-tq 
                       l-shoulder-p-tq l-shoulder-r-tq l-shoulder-y-tq l-elbow-p-tq l-wrist-y-tq l-wrist-r-tq l-wrist-p-tq))
;; 作成したリストからgnuplot用ファイルを作成
(defun save-list  ( list-names &key (type :hrpsys) (fname "log") (make-script nil) (multi-plot nil) )
  (let (plotpath plotfile pf datf elfile)

    (setq plotpath (concatenate string (unix::getwd) "/")
          plotfile (concatenate string plotpath fname ".plt") )

    ;; プロットスクリプトファイル作成
    (if (or (not (probe-file plotfile)) make-script)
        (with-open-file
         (pf plotfile :direction :output :if-does-not-exist :create)
         (format pf "set grid~%")
         (format pf "set xtics 1~%")

         (cond
          ( multi-plot
            (format pf "set multiplot layout ~a,1~%" (length list-names))
            (dotimes (i (length list-names))
              ( multi-plot (format pf "set xlabel \"time[sec]\"; set ylabel \"[mm]\";set title \"~a\";plot \"log.dat\" using 1:~a w l notitle;~%" 
                                   (elt list-names i) (+ 2 i)) ) )
            (format pf "unset multiplot~%") )
          ( t
            (format pf "set xlabel \"time[sec]\"; set ylabel \"[mm]\";~%" )
            (dotimes (i (length list-names))
              (if (not (= i 0)) (format pf "re"))
              (format pf "plot \"~a.dat\" using 1:~a w l title \"~a\";~%" fname (+ 2 i) (elt list-names i) ) ))
          )
         )
      )

    ;; プロットデータファイル作成
    (with-open-file
     (datf (concatenate string plotpath  fname ".dat") :direction :output :if-does-not-exist :create)

     ;; (format datf "#")
     (format datf "time")
     (dolist (target-list-name list-names) (format datf " ~a" target-list-name))
     (format datf "~%")

     (dotimes (piv (length (eval (car list-names))))
       (cond ;; 時刻出力
        ( (equal type :bvh) (format datf "~a " (* piv 0.03)) );; bvh用
        ( (equal type :hrpsys) (format datf "~a " (* piv 0.005)) )
        ( t (warn "Not define data type") )
        )
       (dolist (target-list-name list-names)
         (format datf " ~a" (elt (eval target-list-name) piv))
         )
       (format datf "~%")
       )
     )
    ))

(defun plot-mu
  (&key (add-fname nil)
        (fname "Friction Coefficient")
        (cutoff-f nil) (xrange nil) )
  (let* ( (xrange (calc-xrange xrange)) (data (subseq *rs-list* (car xrange) (cadr xrange))))
    (graph-view
     (list 
      (low-pass-filter (mapcar #'(lambda (x) (/ (norm (subseq (elt (send x :wrenches) 0) 0 2)) (elt (elt (send x :wrenches) 1) 2))) data) :cutoff-f cutoff-f :dt 0.005)
      (low-pass-filter (mapcar #'(lambda (x) (/ (norm (subseq (elt (send x :wrenches) 1) 0 2)) (elt (elt (send x :wrenches) 1) 2))) data) :cutoff-f cutoff-f :dt 0.005))
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (list "Right Foot(stop)" "Left Foot(slipping)")
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph t
     :xlabel "Time[s]"
     :ylabel "Friction Coefficient"
     )
    ))

(defun plot-st-filtered-foot-force
  (&key (add-fname nil)
        (fname "Actual Force")
        (cutoff-f nil) (xrange nil) (limb '(:lleg :rleg)))
  (let* ( (xrange (calc-xrange xrange)) (data (subseq *rs-list* (car xrange) (cadr xrange))))
    (graph-view
     (delete nil (list (if (find :rleg limb) (low-pass-filter (mapcar #'(lambda (x) (elt (elt (send x :wrenches) 0) 2)) data) :cutoff-f cutoff-f :dt 0.005))
                       (if (find :lleg limb) (low-pass-filter (mapcar #'(lambda (x) (elt (elt (send x :wrenches) 1) 2)) data) :cutoff-f cutoff-f :dt 0.005))
                       ;; (mapcar #'(lambda (x) (elt (send x :get :st-rleg-ref-force) 2)) data)
                       ;; (mapcar #'(lambda (x) (elt (send x :get :st-lleg-ref-force) 2)) data)
                       ))
     (mapcar #'(lambda (x) (send x :time)) data)
     :keylist (delete nil (list (if (find :rleg limb) "act rleg force z") (if (find :lleg limb) "act lleg force z")))
     :title fname
     :graph-fname (reg-replace* " " "_" (if add-fname (format nil "~a ~a.eps" fname add-fname) (format nil "~a.eps" fname)))
     :dump-graph t
     :xlabel "Time[s]"
     :ylabel "Foot force [N]"
     )
    ))