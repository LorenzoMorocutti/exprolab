(define (problem task)
(:domain domain_exprob)
(:objects
    robot - agent
    w1 w2 w3 w4 - waypoint
    start - home
)
(:init

    (ishome robot start)




    (notvisited w1)
    (notvisited w2)
    (notvisited w3)
    (notvisited w4)

)
(:goal (and
    (checked_result)
))
)
