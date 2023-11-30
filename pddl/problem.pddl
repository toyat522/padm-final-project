(define 
    (problem organize) 
    (:domain kitchen)

    (:objects 
        sugar - box
        spam - box
        burner - static
        countertop - static
        drawer - openable
        cabinet - openable
    )

    (:init
        (placed sugar burner)
        (placed spam countertop)
        (not_placed_any drawer)
        (not_placed_any cabinet)
        (not_grasping_any)
        (closed drawer)
        (closed cabinet)
    )

    (:goal (and
        (placed sugar countertop)
        (placed spam drawer)
        (closed drawer)
        (closed cabinet)
    ))
)
