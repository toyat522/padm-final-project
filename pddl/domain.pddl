(define 
    (domain kitchen)
    (:requirements :strips :typing)

    (:types
        storage box - object
    )

    (:predicates
        (not_grasping_any)
        (grasping ?b - box)
        (not_placed_any ?s - storage)
        (placed ?b - box ?s - storage)
    )

    (:action pick_up
        :parameters (?b - box ?s - storage)
        :precondition (and 
            (not_grasping_any)
            (placed ?b ?s)
        )
        :effect (and 
            (not (placed ?b ?s))
            (not_placed_any ?s)
            (not (not_grasping_any))
            (grasping ?b)
        )
    )

    (:action place
        :parameters (?b - box ?s - storage)
        :precondition (and 
            (not_placed_any ?s)
            (grasping ?b)
        )
        :effect (and 
            (not (grasping ?b))
            (not_grasping_any)
            (not (not_placed_any ?s))
            (placed ?b ?s)
        )
    )
)