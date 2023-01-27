(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "navigation_pr2")
(load "package://pr2eus/pr2-interface.l")
(load "package://navigation_pr2/euslisp/tuckarm-with-hand.l")

(when (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

(ros::roseus "move_wrist")

(defun give-hand-pose-tmp ()
  (pr2-tuckarm-pose)
  (send *pr2* :larm :angle-vector #f(63.1013 37.3441 -37.2423 -117.344 -140.472 -114.592 49.8059))
  (send *pr2* :head :angle-vector #f(77.0 38.0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  t)

(defun give-hand-pose (hand-coords)
  ;; (pr2-tuckarm-pose :larm :inside :single)
  (pr2-tuckarm-pose)
  (send *ri* :go-pos-unsafe (/(- (elt (send hand-coords :pos) 0) -135) 1000) (/(+ (elt (send hand-coords :pos) 1) 679) 1000) (rad2deg pi))
  
  ;; (send *pr2* :larm :angle-vector #f(44.3333 36.9321 9.03022 -121.342 -133.409 -92.8499 -19.1599))
  (send *pr2* :larm :angle-vector #f(63.1013 37.3441 -37.2423 -117.344 -140.472 -114.592 49.8059))
  (send *pr2* :head :angle-vector #f(77.0 38.0))

  ;; (setq hand-height (elt (send hand-coords :pos) 2))
  ;; (setq torso-height (+ 50.0 (/ (- hand-height 812.755) 10)))
  ;; (send *pr2* :torso :angle-vector (float-vector torso-height))
  ;; (send *pr2* :torso :angle-vector #f(208.957))
  
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  t)

(defun hand-pose-subscriber ()
  (send *pr2* :head :angle-vector #f(0.0 23.226))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :head-controller)
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

(defun execute-cb (server goal)
  (let ((hand-coords nil)
        (start (ros::time-now)))
    ;; (while (or (null hand-coords)
    ;;            (> (elt (send hand-coords :pos) 2) 1500))
    ;;   (ros::spin-once)
    ;;   (when (send server :is-preempt-requested)
    ;;     (send server :set-preempted)
    ;;     (return-from execute-cb nil))
    ;;   (ros::ros-info "cb")
    ;;   (setq hand-coords (hand-pose-subscriber))
    ;;   (when hand-coords
    ;;     (ros::ros-info "~A" hand-coords)))
    ;; (give-hand-pose hand-coords)
    (give-hand-pose-tmp)
    (send server :set-succeeded (send server :result :result t))))
    
(ros::roseus-add-msgs "navigation_pr2")
(setq *as*
      (instance ros::simple-action-server :init
                "/moving_wrist" navigation_pr2::MoveWristAction
                :execute-cb #'execute-cb))

(ros::rate 10)
(ros::ros-info "start")
(do-until-key
 (send *as* :worker)
 (ros::spin-once)
 (ros::sleep))
