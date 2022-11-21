(ros::roseus-add-msgs "geometry_msgs")
(load "package://teach_spot/euslisp/marker.l")
(load "package://pr2eus/pr2-interface.l")
(load "package://navigation_pr2/euslisp/utils.l")
(ros::roseus-add-msgs "navigation_pr2")
(ros::roseus-add-srvs "navigation_pr2")

(ros::roseus "spot_map_server")
(pr2-init nil)

(setq *tfl* (instance ros::transform-listener :init))
(setq *scene* (make-scene-by-tf *tfl*))

;; server
(defclass map-server
  :slots (record-spot-action spot-graph spot-list prev-spot spot-label))
  ;; :slots (label mode spot-msg spot-graph spot-list path prev-n goal-n goal-name sol))

(defmethod map-server
  (:init ()
    (setq spot-graph (instance graph :init))
    (setq spot-list nil)
    (setq *auto-map* nil)
    (setq record-spot-action
         (instance ros::simple-action-server :init "record_spot"
                   navigation_pr2::RecordSpotAction
                   :execute-cb `(lambda-closure nil 0 0 (server goal)
                                                (send ,self :add-spot-cb server goal))))
    (ros::advertise-service "start_auto_map" std_srvs::Empty #'send self :start-auto-map)
    (ros::advertise-service "stop_auto_map" std_srvs::Empty #'send self :stop-auto-map)
    (ros::advertise-service "list_spot_name" navigation_pr2::ListSpotName #'send self :list-spot-name)
    (setq spot-label 0)

     ;; (setq sol (instance breadth-first-graph-search-solver :init))
     (ros::ros-info "init ended"))

  (:start-auto-map (req)
                   (setq *auto-map* t)
                   (ros::ros-info "start auto map")
                   (send req :response))

  (:stop-auto-map (req)
                  (setq *auto-map* nil)
                  (ros::ros-info "stop auto map")
                  (send req :response))

  (:list-spot-name (req)
                   (let ((res (send req :response))
                         (name-list nil)
                         (floor-list nil))
                     (dolist (i (send spot-graph :nodes))
                       (push (send i :get :name-jp) name-list)
                       (push (send i :get :floor) floor-list)
                       )
                     (send res :names name-list)
                     (send res :floors floor-list)
                     res))

  (:add-spot-cb (server goal)
                (let ((command (send goal :goal :command)))
                  (cond
                   ((eq command 0)
                    (setq current-coords (send *ri* :state :worldcoords))
                    (send self :add-spot current-coords)
                    )
                   ((eq command 1)
                    (let ((name (send goal :goal :name))
                          (name-jp (send goal :goal :name_jp))
                          (current-coords (send *ri* :state :worldcoords)))
                      (send self :add-spot current-coords (cons name name-jp))
                      ))
                   ((eq command 2)
                    (send self :remove-spot (send goal :goal :name_jp))
                    ))
                  (setq msg (send server :result :result t))
                  (send server :set-succeeded msg)))
                               
  (:add-spot (coords &optional (name nil))
             (unless (or (null prev-spot) (null name))
               (when (< (abs (coords-difference coords (send prev-spot :get :coords))) 0.25)
                 (unless (send prev-spot :get :name-jp)
                   (ros::ros-info (format nil "rename ~A to ~A" (send prev-spot :name) (car name)))
                   (send prev-spot :put :coords coords)
                   (send prev-spot :name (car name))
                   (send prev-spot :put :name-jp (cdr name))
                   (push (cdr name) spot-list)
                   (return-from :add-spot))))
             
             (dolist (i (send spot-graph :nodes))
               (unless (string= (send i :name) (send prev-spot :name))
                 (let ((diff (abs (coords-difference coords (send i :get :coords)))))
                   (when  (< diff 0.40)
                     (unless (and (send i :get :name-jp) name)
                       (ros::ros-info "diff: ~A" diff)
                       (when prev-spot
                         (send spot-graph :add-arc-from-to prev-spot i :both t)
                         (ros::ros-info "arc from ~A to ~A" (send prev-spot :name) (send i :name)))
                       (when name
                         (ros::ros-info "rename ~A to ~A" (send i :name) (car name)) ;;rename
                         (send i :name (car name))
                         (send i :put :name-jp (cdr name))
                         (push (cdr name) spot-list))
                       (setq prev-spot i)
                       (return-from :add-spot))))))
             (ros::ros-info "~A" name)
             (let (n)
               (setq n (instance node :init (or (if name (car name) nil)
                                                (format nil "~A" spot-label))))
               (when name
                 (send n :put :name-jp (cdr name))
                 (push (cdr name) spot-list))
               (unless name
                 (incf spot-label))
               (send n :put :coords coords)
               (send n :put :floor (check-current-floor *scene* *tfl*))
               (ros::ros-info "node ~A floor:~A~% coords:~A" (send n :name) (send n :get :floor) (send n :get :coords))
               (send spot-graph :add-node n)

               (when prev-spot
                 (send spot-graph :add-arc-from-to prev-spot n :both t)
                 (ros::ros-info "arc from ~A to ~A" (send prev-spot :name) (send n :name)))
               (setq prev-spot n)
               ))

  (:remove-spot (name-jp)
                (dolist (i (send spot-graph :nodes))
                  (when (string= (send i :name-jp) name-jp)
                    (send spot-graph :remove-node i)
                    (remove name-jp spot-list)))
               )


  ;; (:voice-cb (msg)


  ;;                ((substringp "に連れて行って" text)
  ;;                 (send *ri* :speak-jp "はい")
  ;;                 (setq goal-name (rstrip "nitsureteitte" roman-text))
  ;;                 (setq goal-n (send spot-graph :node goal-name))
  ;;                 (ros::ros-info "goal-node: ~A" (send goal-n :name))
  ;;                 (when goal-n
  ;;                   (when (send goal-n :get :name-jp)
  ;;                     (send *ri* :speak-jp (format nil "~Aに行きます" (send goal-n :get :name-jp))))
  ;;                   (let ((spot-coords spot-msg) start-n)
  ;;                     ;; (dolist (n (send spot-graph :nodes)) ;; if node exists nearby
  ;;                     ;;   (when (< (abs (coords-difference spot-coords (send n :get :coords))) 0.3)
  ;;                     ;;     (setq start-n n)
  ;;                     ;;     (return)))
  ;;                     (setq start-n prev-n)
  ;;                     (ros::ros-info "start-node: ~A" start-n)
  ;;                     (ros::ros-info "path0:~A" (length path))
  ;;                     (when start-n
  ;;                       (setq path nil)
  ;;                       (send spot-graph :start-state start-n)
  ;;                       (send spot-graph :goal-state goal-n)
  ;;                       (setq path (send sol :solve spot-graph))
  ;;                       (send sol :clear-open-list)
  ;;                         (return-from :voice-cb)))))

  ;;                 (ros::ros-info "solver failed")
  ;;                 (send *ri* :speak-jp "道がわかりません")
  ;;                 )

  ;;                ((substringp "の行き方を教えて" text)
  ;;                 (setq goal-name (rstrip "の行き方を教えて" text))
  ;;                 (setq goal-n (send spot-graph :node goal-name))
  ;;                 (ros::ros-info "goal-node: ~A" (send goal-n :name))
  ;;                 (when goal-n
  ;;                   (let ((spot-coords spot-msg) start-n)
  ;;                     (setq start-n prev-n)
  ;;                     (ros::ros-info "start-node: ~A" (send start-n :name))
  ;;                     (when start-n
  ;;                       (setq path nil)
  ;;                       (setq path (send sol :solve-by-name spot-graph (send start-n :name) goal-name))
  ;;                       (when path
  ;;                         (dolist (i path)
  ;;                           (ros::ros-info (format nil "~A" (send (send i :state) :name))))
  ;;                         ;; (send *ri* :speak-jp "の順に通ります" :wait t)
  ;;                         (return-from :voice-cb))))
                    ;; (ros::ros-info "solver failed")))
                 
                 ;; ((and (substringp "ここが" text) (substringp "だよ" text))
                 ;;  (let (spot-name (spot-coords spot-msg))
                 ;;    (setq spot-name (list (lstrip "kokoga" (rstrip "dayo" roman-text))
                 ;;                          (lstrip "ここが" (rstrip "だよ" text))))
                 ;;    (unless (null spot-coords)
                 ;;      (ros::ros-info "spot-name: ~A" (cadr spot-name))
                 ;;      (push (cadr spot-name) spot-list)
                 ;;      (send *ri* :speak-jp (format nil "~Aですね" (cadr spot-name)))
                 ;;      (unless (null prev-n)
                 ;;        (when (< (abs (coords-difference spot-coords (send prev-n :get :coords))) 0.25)
                 ;;          (unless (send prev-n :get :name-jp)
                 ;;            (ros::ros-info (format nil "rename ~A to ~A" (send prev-n :name) (car spot-name)))
                 ;;            (send prev-n :put :coords spot-coords)
                 ;;            (send prev-n :name (car spot-name))
                 ;;            (send prev-n :put :name-jp (cadr spot-name))
                 ;;            (return-from :voice-cb))))
                 ;;      (send self :add-spot spot-coords spot-name))
                 ;;    ))
                 
                 ;; )))
  
  (:run ()
    (send record-spot-action :worker)
    ;; 自動保存
    (when *auto-map*
      (setq curr-coords (send *ri* :state :worldcoords))
      (when (null prev-spot)
        (send self :add-spot curr-coords))
      (unless (null prev-spot)
        (setq prev-coords (send prev-spot :get :coords))
        (setq prev-v (send  prev-coords :rotate-vector #f(1 0 0)))
        (setq curr-v (send curr-coords :rotate-vector #f(1 0 0)))
        (setq angle (counter-clockwise-angle-between-vectors prev-v curr-v #f(0 0 1)))
        (when (or (and (and (> (rad2deg angle) 40)
                            (< (rad2deg angle) 320))
                       (> (abs (coords-difference curr-coords prev-coords)) 0.3))
                  (> (abs (coords-difference curr-coords prev-coords)) 0.8))
          (send self :add-spot curr-coords))))
    ;; markerをpublishする
    (let* (spots (stamp (ros::time-now)) (header (instance std_msgs::header :init :stamp stamp :frame_id "/world")))
      (dolist (i (send spot-graph :nodes))
        (push (cons (send i :name) (send i :get :coords)) spots))
      (publish-pin-marker-array spots header)
      (publish-arcs-marker-array (send spot-graph :nodes) header))
    )
  )

(setq *ms* (instance map-server :init))
(ros::rate 20)
(do-until-key
 (ros::spin-once)
 (send *ms* :run)
 (ros::sleep))
