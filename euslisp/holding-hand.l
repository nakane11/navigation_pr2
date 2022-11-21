(ros::roseus-add-msgs "jsk_recognition_msgs")
(load "package://pr2eus/pr2-interface.l")

(when (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

(defun give-hand-pose (hand-coords)
  (if (> (elt (send hand-coords :pos) 1) 0)
      (setq arm "larm")
      (setq arm "rarm"))
  (if (string-equal arm "larm")
      (progn
        (pr2-tuckarm-pose :larm :inside :single)
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) -135) 1000) (/(+ (elt (send hand-coords :pos) 1) 679) 1000) (rad2deg pi)))
      (progn
        (pr2-tuckarm-pose :rarm :inside :single)
        (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) -135) 1000) (/(- (elt (send hand-coords :pos) 1) 679) 1000) (rad2deg pi))))
  (if (string-equal arm "larm")
      (progn
        (send *pr2* :larm :angle-vector #f(44.3333 36.9321 9.03022 -121.342 -133.409 -92.8499 -19.1599))
        (send *pr2* :head :angle-vector #f(77.0 23.0))
        )
      (progn
        (send *pr2* :rarm :angle-vector #f(-44.3333 36.9321 9.03022 -121.342 133.409 -92.8499 -19.1599))
        (send *pr2* :head :angle-vector #f(-77.0 23.0))
        ))

  (setq hand-height (elt (send hand-coords :pos) 2))
  (setq torso-height (+ 50.0 (/ (- hand-height 812.755) 10)))
  (send *pr2* :torso :angle-vector (float-vector torso-height))

  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  t)

(defun hand-pose-subscriber ()
  (send *pr2* :head :angle-vector #f(0.0 14.694))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (setq hand-pose-array (one-shot-subscribe "/hand_pose_estimation_2d/output/pose" jsk_recognition_msgs::HandPoseArray))
  (when (> (length (send hand-pose-array :poses)) 0)
    (setq hand-pose (car (send hand-pose-array :poses)))
    (setq finger-names (send hand-pose :finger_names))
    (when (string= (car finger-names) "wrist")
      (setq stamp (send hand-pose-array :header :stamp))
      (setq base-to-camera-coords (send *tfl* :lookup-transform "/base_link" (send hand-pose-array :header :frame_id) stamp))
      (when base-to-camera-coords
        (setq camera-to-hand-coords (ros::tf-pose->coords (car (send hand-pose :poses))))
        (setq base-to-hand (send base-to-camera-coords :transform camera-to-hand-coords))
        (return-from hand-pose-subscriber base-to-hand)
        )))
  nil)

(pr2-init)
(setq hand-coords nil)
(setq start (ros::time-now))
(while (or (null hand-coords)
           (> (elt (send hand-coords :pos) 2) 1500))
  (ros::spin-once)           
  (ros::ros-info "cb")
  (setq hand-coords (hand-pose-subscriber)))
(setq end (ros::time-now))
(ros::ros-info (format nil "time: ~A" (send (ros::time- end start) :to-sec)))
(give-hand-pose hand-coords)


