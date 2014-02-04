(ns vindinium.astar-test
  (:require [clojure.test :refer :all]
            [vindinium.astar :refer :all]))

(defn- xor [x y]
  (and (or x y)
       (not (and x y))))

(defn- manhattan-distance [graph [x1 y1] [x2 y2]]
  (+ (Math/abs (- x1 x2)) 
     (Math/abs (- y1 y2))))

(defn- h [graph start goal]
  (manhattan-distance graph start goal))

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
                     (xor (= x nx) (= y ny)))]
      [nx ny])))

(defmacro with-private-fns [[ns fns] & tests]
  "Refers private fns from ns and runs tests in context."
  `(let ~(reduce #(conj %1 %2 `(ns-resolve '~ns '~%2)) [] fns)
     ~@tests))



(def simple-graph [[0 0 0 0]
                   [1 1 1 1]
                   [2 2 2 2]
                   [3 3 3 3]])

; Tests for reconstruct-path
(with-private-fns [vindinium.astar [reconstruct-path]]

  (deftest test-reconstruct-path
    (is (= [[1 1] [0 1] [0 0]] 
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
    (is (= rslt  (manhattan-distance [] start finish)))))

(deftest test-find-neighbors-top-left-corner
  (is (= (sort [[1 0] [0 1]])
         (sort (find-neighbors simple-graph [0 0])))))

(deftest test-find-neighbors-bottom-right-corner
  (is (= (sort [[2 3] [3 2]])
         (sort (find-neighbors simple-graph [3 3])))))

(deftest test-find-neighbors-middle
  (is (= (sort (for [x (range 3) y (range 3) :when (xor (= x 1) (= y 1))] [x y]))
         (sort (find-neighbors simple-graph [1 1])))))


(deftest test-astar-simple
  (let [graph [[0 0 0 0]
               [0 0 0 0]
               [0 0 0 0]
               [0 0 0 0]]
        path (a-star [0 0]
                     [3 3]
                     (partial manhattan-distance graph)
                     (partial find-neighbors graph))]
    (println path)))
