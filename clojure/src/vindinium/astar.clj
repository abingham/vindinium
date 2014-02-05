(ns vindinium.astar)

;-------------------------------------------------------------------------------

(defrecord State [g-score 
                  f-score 
                  closed-set 
                  open-set
                  came-from])

(defrecord GoalDef [goal
                    heuristic
                    find-neighbors])

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
                     {goal      :goal
                      heuristic :heuristic}
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

(defn- calc-new-state [{open-set   :open-set
                        closed-set :closed-set
                        :as state}
                       current
                       graph]
  "Given a processing STATE, a CURRENT node, a GOAL node, a distance
HEURISTIC, and a function to FIND-NEIGHBORS, this will calculate a new
processing state."
  (let [intermediate-state (assoc state 
                             :open-set (disj open-set current)
                             :closed-set (conj closed-set current))
        neighbors (filter #(not (contains? (:closed-set intermediate-state) %)) 
                          ((:find-neighbors graph) current))
        state-updates (map #(update-state current 
                                          % 
                                          graph
                                          intermediate-state)
                           neighbors)]
    (reduce 'combine-states 
            intermediate-state
            state-updates)))

(defn a-star 
  "Uses A-* to find a shortest path from a start node to a goal node in a graph.
  
  ``start`` is the \"index\" of the start node in the graph. It can be
  any type, so long as it's understood by the ``heuristic`` and
  ``find-neighbor`` functions.

  ``goal`` is the index of the goal node in the graph. It has the same
  constraints as ``start``.

  ``heuristic`` is an *admissible* heuristic function for finding the
  distance between two nodes in the graph. *Admissible* means that it
  never over-estimates the distance. It is a function of two arguments
  ``([node-a node-b])``.

  ``goal-def`` must by a ``GoalDef`` record, and its
  ``find-neighbors`` function must find all traversable neighbors of a
  node. Its signature is ``([node])`` and it must return a sequence of
  neighbors.

  If a path from ``start`` to ``goal`` exists, then this returns an
  in-order sequence of nodes (the type of which is determined by
  ``find-neighbors``) - starting with ``start`` and ending with
  ``goal`` - describing the path. If no path exists, this returns an
  empty sequence.
"
    [start
     ^GoalDef {goal      :goal
               heuristic :heuristic
               :as goal-def}]

  (loop [state (make-initial-state start (heuristic start goal))
         current start]
    (if (= current goal) 
      (reconstruct-path (:came-from state) goal)
      (let [new-state (calc-new-state state current goal-def)]
        (if (empty? (:open-set new-state))
          []
          (recur new-state
                 (apply min-key 
                        (:f-score new-state) 
                        (:open-set new-state))))))))
