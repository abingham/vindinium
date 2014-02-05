(ns vindinium.astar)

;-------------------------------------------------------------------------------

(defrecord State [g-score 
                  f-score 
                  closed-set 
                  open-set
                  came-from])

(defn- make-initial-state [start h-to-goal]
  (let [g-score (hash-map start 0)
        f-score (hash-map start h-to-goal)
        closed-set (hash-set)
        open-set (hash-set start)
        came-from (hash-map)]
    (->State g-score
             f-score
             closed-set
             open-set
             came-from)))

(defn- reconstruct-path [came-from node]
  (loop [node node
         path [node]]
    (if (not (contains? came-from node))
      (reverse path)
      (recur (came-from node) 
             (conj path (came-from node))))))

(defn- update-state [current
                    n
                    goal
                    heuristic
                    {g-score    :g-score
                     f-score    :f-score
                     open-set   :open-set
                     closed-set :closed-set
                     came-from  :came-from
                     :as state}]
  "Updates the algorithm state (g-score, f-score, open-set, and
came-from) based on a discovering a neighber N of node CURRENT."
  (let [tentative-score 
        ; NOTE: This assumes a distance of 1 from a node to any
        ; neighbor (as defined by the user-supplied neighbor
        ; function.)
        (inc (g-score current))]
    (if (or (not (contains? g-score n)) 
            (< tentative-score (g-score n)))
      (assoc state :g-score (assoc g-score n tentative-score)
                   :f-score (assoc f-score n (+ tentative-score (heuristic n goal)))
                   :open-set (conj open-set n)
                   :came-from (assoc came-from n current))
      state)))

(defn- combine-states [x y]
  (into {} (for [key (keys x)]
             [key (conj (x key) (y key))])))

(defn- calc-new-state [{open-set :open-set
                        closed-set :closed-set
                        :as state}
                       current
                       goal
                       heuristic
                       find-neighbors]
  "Given a processing STATE, a CURRENT node, a GOAL node, a distance
HEURISTIC, and a function to FIND-NEIGHBORS, this will calculate a new
processing state."
  (let [intermediate-state (assoc state 
                             :open-set (disj open-set current)
                             :closed-set (conj closed-set current))
        neighbors (filter #(not (contains? (:closed-set intermediate-state) %)) 
                          (find-neighbors current))
        state-updates (map #(update-state current 
                                          % 
                                          goal 
                                          heuristic
                                          intermediate-state)
                           neighbors)]
    (reduce 'combine-states 
            intermediate-state
            state-updates)))

(defn a-star [start 
              goal
              heuristic
              find-neighbors]
  "Uses A-* to find a shortest path from node START to node
GOAL. HEURISTIC must be an admissible (non-overestimating) function
taking two nodes and returning an estimate of the distance from one to
the other. FIND-NEIGHBORS must take a node and return all of its
neighbors in the graph."
  (loop [state (make-initial-state start (heuristic start goal))
         current start]
    (if (= current goal) 
      (reconstruct-path (:came-from state) goal)
      (let [new-state (calc-new-state state current goal heuristic find-neighbors)]
        (if (empty? (:open-set new-state))
          []
          (recur new-state
                 (apply min-key 
                        (:f-score new-state) 
                        (:open-set new-state))))))))
