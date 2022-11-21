(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")
(require :eng8-scene "package://jsk_maps/src/eng8-scene.l")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun make-scene-by-tf (tfl)
  "Load building scene by existing TF frame"
  (let (frames scene)
    (unix::sleep 1) ;; wait for TF messages

    (setq frames (send tfl :get-frame-strings))
    (cond ((find "eng2" frames :test #'string=)
           (setq scene (make-eng2-scene)))
          ((find "eng8" frames :test #'string=)
           (setq scene (make-eng8-scene)))
          (t
           (ros::ros-fatal
            "There is no building frame_id in TF tree, so exiting...")
           (exit 1))
          )
    scene))

(defun transform-pose-to-target-frame (coords frame-id tfl
                                       &optional (tm (ros::time-now))
                                                 (timeout 1.0))
  (send tfl :wait-for-transform frame-id (send coords :name) tm timeout)
  (let ((tra (send tfl :lookup-transform frame-id (send coords :name) tm)))
    (when tra
      (send tra :transform coords)
      (send tra :name frame-id)
      tra)))

(defun check-current-floor (scene tfl &optional pose (tm (ros::time-now)))
  ;; :name == frame_id
  (if pose
      (setq pose (transform-pose-to-target-frame pose (send scene :name) tfl))
    (progn
      (send tfl :wait-for-transform (send scene :name) "base_footprint" tm 1.0)
      (setq pose (send tfl :lookup-transform
                       (send scene :name) "base_footprint" tm))))
  (send scene :current-floor pose))

(defun lstrip (bag str)
  (declare (string str))
  (declare (string bag))
  (if (string= (subseq str 0 (length bag)) bag)
      (subseq str (length bag))
      nil))

(defun rstrip (bag str)
  (declare (string str))
  (declare (string bag))
  (if (string= (subseq str (- (length str) (length bag))) bag)
      (subseq str 0 (- (length str) (length bag)))
      nil))

(defun counter-clockwise-angle-between-vectors (v1 v2 normal-vector)
  (let* ((det (v. normal-vector (v* v1 v2)))
         (dot (v. v1 v2))
         (angle (+ (atan2 (- det) (- dot)) pi)))
    (when (eps= angle 2pi)
      (setq angle 0.0))
    angle))

(defun coords-difference (a b)
  (/ (norm (send (send a :transformation b) :pos)) 1000))

