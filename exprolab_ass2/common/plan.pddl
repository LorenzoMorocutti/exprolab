Number of literals: 15
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 10.000
b (9.000 | 2.001)b (8.000 | 2.002)b (7.000 | 3.003)b (6.000 | 3.004)b (5.000 | 4.005)b (4.000 | 4.006)b (3.000 | 5.007)b (2.000 | 6.008)b (1.000 | 7.009);;;; Solution Found
; States evaluated: 21
; Cost: 8.010
; Time 0.01
0.000: (move_away_from_home robot w1 start)  [1.000]
1.001: (check_hint w1 robot)  [1.000]
1.002: (move_to w1 w2 robot)  [1.000]
2.003: (check_hint w2 robot)  [1.000]
2.004: (move_to w2 w3 robot)  [1.000]
3.005: (check_hint w3 robot)  [1.000]
3.006: (move_to w3 w4 robot)  [1.000]
4.007: (check_hint w4 robot)  [1.000]
5.008: (check_complete_hypotesis robot)  [1.000]
6.009: (move_home robot w4 start)  [1.000]
7.010: (check_result robot start)  [1.000]
