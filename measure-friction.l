(load "play-hrpsysfile.l")

(defun move-foot (&key (translate-vec #f(100 0 0)) (yaw-angle 30) (tm 10000) (limb :lleg))
  (let ( (target-coord (send *robot* limb :end-coords :copy-worldcoords))
         (init-pose (send *robot* :angle-vector)) )
    (send *ri* :start-log)

    (send target-coord :translate translate-vec)
    (send target-coord :rotate (deg2rad yaw-angle) :z)
    (send *robot* limb :inverse-kinematics target-coord)
    (setq target-pose (send *robot* :angle-vector))

    (send *ri* :angle-vector (send *robot* :angle-vector) tm)
    (send *ri* :wait-interpolation)
    (send *ri* :angle-vector (send *robot* :angle-vector init-pose) tm)
    (send *ri* :wait-interpolation)

    (save-log :fname (format nil "move-foot_~s,~s,~svec_~syaw_~stm" (elt translate-vec 0) (elt translate-vec 1) (elt translate-vec 2) yaw-angle tm))
    ))
