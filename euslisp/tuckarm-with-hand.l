(load "package://pr2eus/pr2-interface.l")
(setq *pr2-tuckarm-pose-larm-free* (list #f(10.9964 63.8568 73.353 -113.496 5.90481 -92.0995 -23.1082)
                                                #f(-6.6127 60.5828 -122.994 -74.8254 56.2071 -5.72958 10.8427)))

(defun pr2-tuckarm-pose (&optional free-arm (arm-side :both))
  (let* ((current-arm (check-tuckarm-pose :thre 40)) ;; nil rarm larm
         (free-arm (or free-arm current-arm :larm))
         (msec 500))
    (when (not (eq current-arm free-arm))
      (progn
        (setq msec 2000)
        (send *pr2* :larm :angle-vector #f( 25 0 0 -121 0 -6 0))
        (send *pr2* :rarm :angle-vector #f(-25 0 0 -121 0 -6 0))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
        (send *ri* :wait-interpolation)
        ))
    ;; 右腕をたたむ
    (send *pr2* :rarm :angle-vector #f(-6.6127 60.5828 -122.994 -74.8254 56.2071 -5.72958 10.8427))
    (send *ri* :rarm-angle-vector (send *pr2* :angle-vector) 500))
  (send *ri* :wait-interpolation)
  (when (eq arm-side :both)
    ;; 左腕をたたむ
    (send *pr2* :larm :angle-vector #f(10.9964 63.8568 73.353 -113.496 5.90481 -92.0995 -23.1082))
    (send *ri* :larm-angle-vector (send *pr2* :angle-vector) 500)
    (send *ri* :wait-interpolation)
    t
    )
  )

