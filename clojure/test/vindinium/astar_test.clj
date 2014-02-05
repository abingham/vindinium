(ns vindinium.astar-test
  (:require [clojure.test :refer :all]
            [vindinium.astar :refer :all]))

(defn- xor [x y]
  (and (or x y)
       (not (and x y))))

(defn- manhattan-distance [[x1 y1] [x2 y2]]
  (+ (Math/abs (- x1 x2)) 
     (Math/abs (- y1 y2))))

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
                     (xor (= x nx) (= y ny))
                     (not= ((graph ny) nx) 1))]
      [nx ny])))

(defn- neighbors? [a b]
  (= (manhattan-distance a b) 1))

(defmacro with-private-fns [[ns fns] & tests]
  "Refers private fns from ns and runs tests in context."
  `(let ~(reduce #(conj %1 %2 `(ns-resolve '~ns '~%2)) [] fns)
     ~@tests))

(defn- valid-path? [path]
  (every? #(apply neighbors? %) (partition 2 1 path)))

(defn- valid-solution? [path start goal]
  (and (= (first path) start)
       (= (last path) goal)
       (valid-path? path)))

(def simple-graph [[0 0 0 0]
                   [0 0 0 0]
                   [0 0 0 0]
                   [0 0 0 0]])

; Tests for reconstruct-path
(with-private-fns [vindinium.astar [reconstruct-path]]

  (deftest test-reconstruct-path
    (is (= [[0 0] [0 1] [1 1]] 
           (reconstruct-path {[1 1] [0 1],
                              [0 1] [0 0]}
                             [1 1]))))

  (deftest test-reconstruct-path-no-path
    (is (= [[2 2]]
           (reconstruct-path {[1 1] [0 1],
                              [0 1] [0 0]}
                             [2 2])))))

; sanity tests for our utility functions

(deftest test-manhattan-distance
  (doseq [[rslt start finish] 
          [[2 [0 0] [1 1]]
           [0 [100 100] [100 100]]
           [20 [5 5] [-5 -5]]
           [20 [-5 5] [5 -5]]]] 
    (is (= rslt  (manhattan-distance start finish)))))

(deftest test-find-neighbors-top-left-corner
  (is (= (sort [[1 0] [0 1]])
         (sort (find-neighbors simple-graph [0 0])))))

(deftest test-find-neighbors-bottom-right-corner
  (is (= (sort [[2 3] [3 2]])
         (sort (find-neighbors simple-graph [3 3])))))

(deftest test-find-neighbors-middle
  (is (= (sort (for [x (range 3) y (range 3) :when (xor (= x 1) (= y 1))] [x y]))
         (sort (find-neighbors simple-graph [1 1])))))

(deftest test-find-neighbors-exclude-right
  (let [graph [[0 1]
               [0 0]]
        neighbors (find-neighbors graph [0 0])]
    (is (= neighbors [[0 1]]))))

(deftest test-find-neighbors-exclude-below
  (let [graph [[0 0]
               [1 0]]
        neighbors (find-neighbors graph [0 0])]
    (is (= neighbors [[1 0]]))))

(deftest test-find-neighbors-exclude-left
  (let [graph [[1 0]
               [0 0]]
        neighbors (find-neighbors graph [1 0])]
    (is (= neighbors [[1 1]]))))

(deftest test-find-neighbors-exclude-above
  (let [graph [[1 0]
               [0 0]]
        neighbors (find-neighbors graph [0 1])]
    (is (= neighbors [[1 1]]))))

(deftest test-astar-simple
  (let [graph [[0 1 0 0]
               [0 0 0 0]
               [0 0 0 0]
               [0 0 0 0]]
        start [0 0]
        goal [3 3]
        path (a-star start
                     goal
                     manhattan-distance
                     (partial find-neighbors graph))]
    (is (valid-solution? path start goal))))

(deftest test-astar-snake
  (let [graph [[0 0 1 1]
               [1 0 1 1]
               [1 0 0 0]
               [1 1 1 0]]
        start [0 0]
        goal [3 3]
        path (a-star start
                     goal
                     manhattan-distance
                     (partial find-neighbors graph))]
    (is (valid-solution? path start goal))))

(deftest test-astar-no-solution
  (let [graph [[0 0 0 0]
               [0 0 0 0]
               [1 1 1 1]
               [0 0 0 0]]
        start [0 0]
        goal [3 3]
        path (a-star start
                    goal
                     manhattan-distance
                     (partial find-neighbors graph))]
    (is (= path []))))
