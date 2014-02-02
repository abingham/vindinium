(ns vindinium.astar-test
  (:require [clojure.test :refer :all]
            [vindinium.astar :refer :all]))

(defmacro with-private-fns [[ns fns] & tests]
  "Refers private fns from ns and runs tests in context."
  `(let ~(reduce #(conj %1 %2 `(ns-resolve '~ns '~%2)) [] fns)
     ~@tests))

(def simple-graph [[0 0 0 0]
                   [1 1 1 1]
                   [2 2 2 2]
                   [3 3 3 3]])

(with-private-fns [vindinium.astar [find-neighbors
                                    manhattan-distance
                                    reconstruct-path]]

  (deftest test-manhattan-distance
    (doseq [[rslt start finish] 
            [[2 [0 0] [1 1]]
             [0 [100 100] [100 100]]
             [20 [5 5] [-5 -5]]
             [20 [-5 5] [5 -5]]]] 
      (is (= rslt  (manhattan-distance [] start finish)))))

  (deftest test-reconstruct-path
    (is (= [[1 1] [0 1] [0 0]] 
           (reconstruct-path {[1 1] [0 1],
                              [0 1] [0 0]}
                             [1 1]))))

  (deftest test-reconstruct-path-no-path
    (is (= [[2 2]]
           (reconstruct-path {[1 1] [0 1],
                              [0 1] [0 0]}
                             [2 2]))))

  (deftest test-find-neighbors-top-left-corner
    (is (= (sort [[1 0] [0 1] [1 1]])
           (sort (find-neighbors simple-graph [0 0])))))

  (deftest test-find-neighbors-bottom-right-corner
    (is (= (sort [[2 3] [2 2] [3 2]])
           (sort (find-neighbors simple-graph [3 3])))))

  (deftest test-find-neighbors-middle
    (is (= (sort (for [x (range 3) y (range 3) :when (not= [1 1] [x y])] [x y]))
           (sort (find-neighbors simple-graph [1 1])))))

)


