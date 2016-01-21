(defun reg-replace-impl (old-pattern new-pattern str delimiter g)
  (read-line (piped-fork (format nil "echo ~a|sed -e 's~a~a~a~a~a~a'" str delimiter old-pattern delimiter new-pattern delimiter g))))

(defun reg-replace* (old-pattern new-pattern str &key (delimiter "#"))
  (reg-replace-impl old-pattern new-pattern str delimiter "g"))

(defun reg-replace (old-pattern new-pattern str &key (delimiter "#"))
  (reg-replace-impl old-pattern new-pattern str delimiter ""))

(defun reg-split (split-str str &key (delimiter "#"))
  (let ( (p (piped-fork (format nil "echo ~a|sed -e 's~a~a~a~a~ag'" str delimiter split-str delimiter "\\n" delimiter)))
         (ret nil) (eof (gensym)) tmp )
    (while (not (eq eof (setq tmp (read-line p nil eof)))) (push tmp ret))
    (reverse ret)
  ))
