(ns vindinium.astar
  (:use clojure.data.priority-map)
  (:use clojure.test))

(defn- manhattan-distance [graph [x1 y1] [x2 y2]]
  (+ (Math/abs (- x1 x2)) 
     (Math/abs (- y1 y2))))

(defn- h [graph start goal]
  (manhattan-distance graph start goal))

(defn- reconstruct-path [came-from node]
  (loop [node node
         path [node]]
    (if (not (contains? came-from node))
      path
      (recur (came-from node) 
             (conj path (came-from node))))))

(defn- graph-size [g]
  {:x (if (empty? g) 0 (count (first g))),
   :y (count g)}
)

(defn- find-neighbors [graph [x y]]
  (let [{size-x :x, size-y :y} (graph-size graph)]
    (for [nx (range (dec x) (+ x 2))
          ny (range (dec y) (+ y 2))
          :when (and (>= nx 0)
                     (>= ny 0)
                     (< nx size-x)
                     (< ny size-y)
                     (not= [nx ny] [x y]))]
      [nx ny])))

(defn update-sets [graph
                   current
                   n
                   goal
                   came-from
                   g-score
                   f-score
                   open-set]
  "Updates the algorithm state (g-score, f-score, open-set, and
came-from) based on a discovering a neighber N of node CURRENT."
  (let [tentative-score (+ (g-score current) (manhattan-distance graph current n))]
    (if (or (not (contains? g-score n)) 
            (< tentative-score (g-score n)))
      
      {:came-from (assoc came-from n current),
       :g-score (assoc g-score n tentative-score),
       :f-score (assoc f-score n (+ tentative-score (h graph n goal)))
       :open-set (conj open-set n)}

      {:came-from came-from, 
       :g-score g-score,
       :f-score f-score,
       :open-set open-set})))

(defn- combine-states [x y]
  ; TODO: Surely this can be done as a comprehension. Look into it.
  {:came-from (conj (x :came-from) (y :came-from))
   :g-score (conj (x :g-score) (y :g-score))
   :f-score (conj (x :f-score) (y :f-score))
   :open-set (conj (x :open-set) (y :open-set))}
)

(defn a-star [graph start goal]
  (loop [g-score (hash-map start 0) ; distance to from start to node
         f-score (hash-map start (h graph start goal)) ; distance to node (g-score) plus heuristic distance to goal
         closed-set (hash-set)
         open-set (sorted-set-by 'f-score start)
         came-from (hash-map)]
    (let [current (first open-set)]
      (if (= current goal) 
        (reconstruct-path came-from goal)
        (let [open-set (disj open-set current)
              closed-set (conj closed-set current)
              neighbors (filter (fn [x] (not (contains? closed-set x))) 
                                (find-neighbors graph current))
              state-updates (map (fn [n] (update-sets graph current n goal came-from g-score f-score open-set))
                                 neighbors)
              new-state (reduce 'combine-states 
                                {:came-from came-from,
                                 :g-score g-score,
                                 :f-score f-score,
                                 :open-set open-set}
                                state-updates)]
          (recur (new-state :g-score)
                 (new-state :f-score)
                 closed-set
                 (new-state :open-set)
                 (new-state :came-from)))))))

;; function A*(start,goal)
;;     closedset := the empty set    // The set of nodes already evaluated.
;;     openset := {start}    // The set of tentative nodes to be evaluated, initially containing the start node
;;     came_from := the empty map    // The map of navigated nodes.
 
;;     g_score[start] := 0    // Cost from start along best known path.
;;     // Estimated total cost from start to goal through y.
;;     f_score[start] := g_score[start] + heuristic_cost_estimate(start, goal)
 
;;     while openset is not empty
;;         current := the node in openset having the lowest f_score[] value
;;         if current = goal
;;             return reconstruct_path(came_from, goal)
 
;;         remove current from openset
;;         add current to closedset
;;         for each neighbor in neighbor_nodes(current)
;;             if neighbor in closedset
;;                 continue
;;             tentative_g_score := g_score[current] + dist_between(current,neighbor)
 
;;             if neighbor not in openset or tentative_g_score < g_score[neighbor] 
;;                 came_from[neighbor] := current
;;                 g_score[neighbor] := tentative_g_score
;;                 f_score[neighbor] := g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
;;                 if neighbor not in openset
;;                     add neighbor to openset
 
;;     return failure
 
;; function reconstruct_path(came_from, current_node)
;;     if current_node in came_from
;;         p := reconstruct_path(came_from, came_from[current_node])
;;         return (p + current_node)
;;     else
;;         return current_node
