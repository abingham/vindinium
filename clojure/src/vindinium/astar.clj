(ns vindinium.astar
  (:use clojure.data.priority-map)
  (:use clojure.test))

;-------------------------------------------------------------------------------

(defn- reconstruct-path [came-from node]
  (loop [node node
         path [node]]
    (if (not (contains? came-from node))
      (reverse path)
      (recur (came-from node) 
             (conj path (came-from node))))))

(defn- update-sets [current
                    n
                    goal
                    heuristic
                    came-from
                    g-score
                    f-score
                    open-set]
  "Updates the algorithm state (g-score, f-score, open-set, and
came-from) based on a discovering a neighber N of node CURRENT."
  (let [tentative-score 
        ; NOTE: This assumes a distance of 1 from a node to any
        ; neighbor (as defined by the user-supplied neighbor
        ; function.)
        (inc (g-score current))]
    (if (or (not (contains? g-score n)) 
            (< tentative-score (g-score n)))
      
      {:came-from (assoc came-from n current),
       :g-score (assoc g-score n tentative-score),
       :f-score (assoc f-score n (+ tentative-score (heuristic n goal)))
       :open-set (conj open-set n)}

      {:came-from came-from, 
       :g-score g-score,
       :f-score f-score,
       :open-set open-set})))

(defn- combine-states [x y]
  (into {} (for [key (keys x)]
             [key (conj (x key) (y key))])))

(defn a-star [start 
              goal
              heuristic
              find-neighbors]
  "Uses A-* to find a path from node START to node GOAL. HEURISTIC
must be function taking two nodes and returning an estimate of the
distance from one to the other. HEURISTIC must be 'admissble' meaning
that it must not overstimate the distance. FIND-NEIGHBORS must take a
node and return all of its neighbors in the graph."
  (loop [g-score (hash-map start 0) ; distance to from start to node
         f-score (hash-map start (heuristic start goal)) ; distance to node (g-score) plus heuristic distance to goal
         closed-set (hash-set)
         open-set (hash-set start)
         came-from (hash-map)]
    ; TODO: There must be a better way to express this than with the if-statements. Pattern matching? Something...
    (if (empty? open-set)
      []
      (let [current (apply min-key 'f-score open-set)]
        (if (= current goal) 
          (reconstruct-path came-from goal)
          (let [open-set (disj open-set current)
                closed-set (conj closed-set current)
                neighbors (filter (fn [x] (not (contains? closed-set x))) 
                                  (find-neighbors current))
                state-updates (map (fn [n] (update-sets current 
                                                        n 
                                                        goal 
                                                        heuristic
                                                        came-from 
                                                        g-score 
                                                        f-score 
                                                        open-set))
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
                   (new-state :came-from))))))))
