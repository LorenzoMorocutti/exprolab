;Header and description

(define (domain domain_exprob)

(:requirements :strips :fluents :durative-actions :typing :negative-preconditions :duration-inequalities)

(:types 
    agent
    waypoint
    home
)

;(:constants )

(:predicates 
    (isAt ?ag - agent ?x - waypoint)
    (isHome ?ag - agent ?h - home)
    (checked ?x - waypoint)
    (checked_complete)
    (checked_result)
    (notvisited ?x - waypoint)
)

(:durative-action move_to
    :parameters (?x1 ?x2 - waypoint ?ag - agent)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (isAt ?ag ?x1)
            (notvisited ?x2)
        ))
    )
    :effect (and 
       (at start (and 
            (not (isAt ?ag ?x1))
            (not (notvisited ?x2))
        ))
        (at end (and 
        
            (isAt ?ag ?x2)
        ))
    )
)

(:durative-action move_home
    :parameters (?ag - agent ?x1 - waypoint ?h - home)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
        (checked_complete)
            (isAt ?ag ?x1)
        ))
    )
    :effect (and 
        (at start (and 
            (not (isAt ?ag ?x1))
        ))
        (at end (and 
            (isHome ?ag ?h)
        ))
    )
)

(:durative-action move_away_from_home
    :parameters (?ag - agent ?x - waypoint ?h - home)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and
            (isHome ?ag ?h) 
            (notvisited ?x)
        ))
    )
    :effect (and 
    (at start (and 
            (not (isHome ?ag ?h))
            (not (notvisited ?x))

        ))
        (at end (and 
            
            (isAt ?ag ?x)
        ))
    )
)


(:durative-action check_hint
    :parameters (?x - waypoint ?ag - agent)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (isAt ?ag ?x)
            ;(not(checked ?x))
        ))
    )
    :effect (and 
        (at end (and 
            (checked ?x)
        ))
    )
)

; (:durative-action check_complete_hypotesis
;     :parameters (?x1 ?x2 ?x3 ?x4 - waypoint)
;     :duration (= ?duration 1)
;     :condition (and 
;         (at start (and 
;             (not(checked_complete))
;             (checked ?x1)
;             (checked ?x2)
;             (checked ?x3)
;             (checked ?x4)
;             (disjuncted ?x1 ?x2 ?x3 ?x4)
;         ))
;     )
;     :effect (and 
;         (at end (and 
;             (checked_complete)
;         ))
;     )
; )

(:durative-action check_complete_hypotesis
    :parameters (?ag - agent)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (forall (?wp - waypoint) ( checked ?wp))
        ))
    )
    :effect (and 
        (at end (and 
            (checked_complete)
        ))
    )
)

(:durative-action check_result
    :parameters (?ag - agent ?h - home)
    :duration (= ?duration 1)
    :condition (and 
        (at start (and 
            (checked_complete)
            (isHome ?ag ?h)
        ))
    )
    :effect (and 
        (at end (and 
            (checked_result)
        ))
    )
)

)