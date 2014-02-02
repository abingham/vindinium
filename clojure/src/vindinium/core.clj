(ns vindinium.core
  (:gen-class)
  (:use [slingshot.slingshot :only [try+, throw+]])
  (:use [clojure.core.match :only (match)])
  (:use 'clojure.java.io)
  (:use 'clojure.string))

(require '[clj-http.client :as http])

(def server-url "http://vindinium.org")

(defn bot [input]
  "Implement this function to create your bot!"
  (with-open [wrtr (writer "foobar")]
    (.write wrtr (board-to-string (-> input :game :board)))
    (.write wrtr "\n\n")
    (.write wrtr (str (-> input :game :board)))
    (.write wrtr "\n\n")
    (.write wrtr (str input)))
  (first (shuffle ["north", "south", "east", "west", "stay"])))

(defn board-to-string [board]
  "Turn a board into a pretty string."
  (clojure.string/join "\n"
                       (map row-to-string (partition (board :size) (board :tiles)))))

(defn row-to-string [row]
  (clojure.string/join "" (map tile-to-char row)))

(defn tile-to-char [tile]
  (match [tile]
   [{:tile :wall}] "*"
   [{:tile :air}] " "
   [{:tile :mine}] "m"
   [{:tile :tavern}] "t"
   [{:tile :hero, :id x}] (str x)
   :else "?"))

(defn parse-tile [tile]
  (match (vec tile)
         [\space \space] {:tile :air}
         [\# \#] {:tile :wall}
         [\[ \]] {:tile :tavern}
         [\$ \-] {:tile :mine}
         [\$ i] {:tile :mine :of i}
         [\@ i] {:tile :hero :id (Integer/parseInt (str i))}))

(defn parse-tiles [tiles] (map parse-tile (partition 2 (seq tiles))))

(defn parse-input [input] (update-in input [:game :board :tiles] parse-tiles))

(defn request [url, params]
  "makes a POST request and returns a parsed input"
  (try+
    (parse-input (:body (http/post url {:form-params params :as :json})))
    (catch map? {:keys [status body]}
      (println (str "[" status "] " body))
      (throw+))))

(defn step [from]
  (loop [input from]
    (print ".")
    (let [next (request (:playUrl input) {:dir (bot input)})]
      (if (:finished (:game next)) (println "") (recur next)))))

(defn training [secret-key turns]
  (let [input (request (str server-url "/api/training") {:key secret-key :turns turns})]
    (println (str "Starting training game " (:viewUrl input)))
    (step input)
    (println (str "Finished training game " (:viewUrl input)))))

(defn arena [secret-key games]
  (loop [it 1]
    (let [p #(println (str "[" it "/" games "] " %))
          _ (p "Waiting for pairing...")
          input (request (str server-url "/api/arena") {:key secret-key})]
      (p (str "Starting arena game " (:viewUrl input)))
      (step input)
      (p (str "Finished arena game " (:viewUrl input)))
      (when (< it games) (recur (+ it 1))))))

(def usage
  "Usage:
   training <secret-key> <number-of-turns>
   arena <secret-key> <number-of-games")

(defn -main [& args]
  (match (vec args)
         ["training", secret-key, nb] (training secret-key nb)
         ["arena", secret-key, nb] (arena secret-key nb)
         :else (println usage)))
