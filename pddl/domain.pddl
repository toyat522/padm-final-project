(define 
    (domain kitchen)
    (:requirements :strips :typing)

    (:types
        storage - object
        box - object
        static - storage
        openable - storage
    )

    (:predicates
        (not_grasping_any)
        (grasping ?b - box)
        (not_placed_any ?s - storage)
        (placed ?b - box ?s - storage)
        (open ?s - storage)
        (closed ?s - storage)
    )

    (:action open_storage
        :parameters (?s - storage)
        :precondition (and 
            (not_grasping_any)
            (closed ?s)
        )
        :effect (and 
            (open ?s)
            (not (closed ?s))
        )
    )

    (:action close_storage
        :parameters (?s - storage)
        :precondition (and 
            (not_grasping_any)
            (open ?s)
        )
        :effect (and 
            (not (open ?s))
            (closed ?s)
        )
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

    (:action place_static
        :parameters (?b - box ?s - static)
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

    (:action place_openable
        :parameters (?b - box ?s - openable)
        :precondition (and 
            (not_placed_any ?s)
            (grasping ?b)
            (open ?s)
        )
        :effect (and 
            (not (grasping ?b))
            (not_grasping_any)
            (not (not_placed_any ?s))
            (placed ?b ?s)
        )
    )
)