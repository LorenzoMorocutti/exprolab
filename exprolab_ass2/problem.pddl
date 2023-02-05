(define (problem problem_exprob) (:domain domain_exprob)
(:objects 
    robot - agent
    start - home
    w1 - waypoint
    w2 - waypoint
    w3 - waypoint
    w4 - waypoint
)

(:init
    (isHome robot start)
    (notvisited w1)
    (notvisited w2)
    (notvisited w3)
    (notvisited w4)
    ; (not(checkedw1))
    ; (not(checked w2))
    ; (not(checked w3))
    ; (not(checked w4))
)

(:goal (and
    ( checked_result)
))

)
