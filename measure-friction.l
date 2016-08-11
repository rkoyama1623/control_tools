(load "play-hrpsysfile.l")
;; (load "package://eus_qp/euslisp/contact-optimization.l")
(load "package://eus_qpoases/euslisp/eus-qpoases.l")

(defun move-foot (&key (translate-vec #f(100 0 0)) (yaw-angle 30) (tm 10000) (fz 100) (nr 0) (np 1) (p 0) (r 0) (limb :lleg) )
  (let ( (target-coord) (init-pose (send *robot* :angle-vector)) (return-pose) (offset-vec #f(0 0 0)) )

    (unix:sleep 5)

    (send *robot* :fix-leg-to-coords (make-coords) :rleg)

    ;; (send *ri* :start-impedance limb :force-gain #f(0 0 0.2) :moment-gain #f(0.8 0.8 0) :m-p 10 :d-p 2500 :k-p 20 :d-r 100 :k-r 20)
    ;; (send *ri* :set-ref-force (float-vector 0 0 fz) 5000 limb)
    ;; ;; (send *ri* :wait-interpolation-seq)
    ;; (send *ri* :set-ref-moment (float-vector nr np 0) 5000 limb)
    ;; (send *ri* :wait-interpolation-seq)
    
    (send *robot* :lleg :ankle-p :joint-angle (+ (send *robot* :lleg :ankle-p :joint-angle) p))
    (send *robot* :lleg :ankle-r :joint-angle (+ (send *robot* :lleg :ankle-r :joint-angle) r))
    (send *ri* :angle-vector (send *robot* :angle-vector))
    (send *ri* :wait-interpolation)


    (setq target-coord (send *robot* limb :end-coords :copy-worldcoords)
          return-pose (send *robot* :angle-vector))

    (send *ri* :start-log)

    (send target-coord :translate translate-vec)
    (send target-coord :rotate (deg2rad yaw-angle) :z)
    (send *robot* limb :inverse-kinematics target-coord)
    (setq target-pose (send *robot* :angle-vector))

    (send *ri* :angle-vector target-pose tm)
    (send *ri* :wait-interpolation)
    (print (send *ri* :state :force-vector))
    (send *ri* :angle-vector return-pose tm)
    (send *ri* :wait-interpolation)
    (print (send *ri* :state :force-vector))

    ;; (send *ri* :stop-impedance :legs)
    ;; (save-log :fname (format nil "move-foot_~s,~s,~svec_~syaw_~stm_~sfz_~snr_~snp" (elt translate-vec 0) (elt translate-vec 1) (elt translate-vec 2) yaw-angle tm fz nr np))
    (save-log :fname (format nil "move-foot_paper_~s,~s,~svec_~syaw_~stm_~sp_~sr" (elt translate-vec 0) (elt translate-vec 1) (elt translate-vec 2) yaw-angle tm p r))

    (send *ri* :angle-vector (send *robot* :angle-vector init-pose))
    (send *ri* :wait-interpolation)

    ))

(defun update-robot-stete (robot-state)
  (send *robot* :angle-vector (send robot-state :angle-vector))
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  )

(defun get-local-force-point-list (left-top right-bottom grid)
  (let* ( (rows (car grid)) (cols (cadr grid))
          (lt-rb (v- right-bottom left-top))
          (x-step (/ (elt lt-rb 0) cols)) (y-step (/ (elt lt-rb 1) rows))
          (force-point-list nil) )
    (dotimes (i (1+ rows))
      (dotimes (j (1+ cols))
        (push (v+ left-top (float-vector (* j x-step) (* i y-step) 0)) force-point-list)
        ))
    (reverse force-point-list)
  ))

(defun get-convert-matrix (cur-pos cur-rot v-dt w-dt mu &key (draw nil) (size 1) (grid))
  (let ( (a-list nil) (b-list nil) (vi-dt) (force-point-list nil) (dxy-list nil)
         ;; (left-top #f(130 50 0)) (right-bottom #f(-120 -90 0));; right foot
         (left-top #f(130 75 0)) (right-bottom #f(-100 -55 0));; left foot
         )
    (dolist (ri-local (get-local-force-point-list left-top right-bottom grid))

      (setq ri (send (make-coords :rot cur-rot) :transform-vector  ri-local);; 3dof
            vi-dt (v+ v-dt (v* w-dt ri));; 3dof
            dxy (subseq (normalize-vector vi-dt) 0 2));; 2dof

      (if draw (send *viewer* :draw-arrow (v+ cur-pos ri) (v+ (v+ cur-pos ri) (scale size vi-dt))))

      (setq r-2dof-closs-vec (float-vector (elt ri 1) (* -1 (elt ri 0))));; 2dof
      (push (transpose (matrix (concatenate float-vector #f(1) r-2dof-closs-vec))) a-list)
      (push (transpose (matrix (scale mu (concatenate float-vector (scale -1 dxy) (float-vector (v. dxy r-2dof-closs-vec)))))) b-list)
      (push (v+ cur-pos ri) force-point-list)
      (push dxy dxy-list)
      )

    ;; (if draw (send *viewer* :flush))
    (list (apply #'concatenate-matrix-row (reverse a-list))
          (apply #'concatenate-matrix-row (reverse b-list))
          (reverse force-point-list)
          (reverse dxy-list))
  ))

(defun estimate-foot-force-distribution (&key (grid '(1 1)))
  (let (last-pos last-rot cur-coord cur-pos cur-rot (dt 0.002) v-dt w-dt (mu 0.1) r-2dof-closs-vec A B force-point-list dxy-list
                 point-num force-vec moment-vec distributed-fz-list)
    (update-robot-stete (car *rs-list*))
    (setq last-pos (send *robot* :lleg :end-coords :worldpos)
          last-rot (send *robot* :lleg :end-coords :worldrot))

    (let ((i 0))
      (dolist (rs *rs-list*)
        (when (= (mod i 20) 0)
          (setq cur-coord (send *robot* :lleg :end-coords :copy-worldcoords)
                cur-pos (send cur-coord :worldpos)
                cur-rot (send cur-coord :worldrot))

          (setq v-dt (v- cur-pos last-pos)
                w-dt (matrix-log (m* cur-rot (inverse-matrix last-rot))))
          (multiple-value-setq (A B force-point-list dxy-list) (get-convert-matrix cur-pos cur-rot v-dt w-dt mu :draw t :size 3 :grid grid))

          (setq point-num (length (matrix-row A 0)))
          ;; (setq force-vec (scale -1 (elt (car (send rs :wrench-list)) 1));; left foot
          ;;       moment-vec (elt (cadr (send rs :wrench-list)) 1));; left foot
          (setq force-vec (send rs :get :lleg-force-vector)
                moment-vec (send rs :get :lleg-moment-vector))
          (format t "force: ~s  moment: ~s~%" force-vec moment-vec)

          (setq distributed-fz-list (coerce (solve-qpoases-qp
                                             :eval-weight-matrix (unit-matrix point-num)
                                             :eval-coeff-vector (make-array point-num :element-type float-vector :initial-element 0)
                                             :inequality-matrix (unit-matrix point-num)
                                             :inequality-min-vector (make-array point-num :element-type float-vector :initial-element 0)
                                             :equality-matrix A
                                             :equality-vector (float-vector (elt force-vec 2) (elt moment-vec 0) (elt moment-vec 1)))
                                            cons))
          ;; draw distributed force
          (mapc #'(lambda (force-point fz dxy)
                    (send *viewer* :draw-arrow force-point (v+ force-point (concatenate float-vector (scale (* mu fz -1) dxy) (float-vector fz)))))
                force-point-list distributed-fz-list dxy-list)

          ;; draw moment
          (dotimes (i 3)
            ;; (send *viewer* :draw-arrow cur-pos (v+ cur-pos (scale (elt moment-vec i) (matrix-column cur-rot i))))
            (send *viewer* :draw-arrow cur-pos (v+ cur-pos (scale (elt (send (make-coords :rot cur-rot) :inverse-transform-vector moment-vec) i) (matrix-column cur-rot i))))
            )

          (setq last-pos cur-pos
                last-rot cur-rot)
          (update-robot-stete rs)
          (send *viewer* :flush)
          (send *viewer* :draw-objects :flush nil)
          )
        (incf i)
        )
      )
    ))
