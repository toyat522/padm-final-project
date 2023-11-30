(define 
    (problem organize) 
    (:domain kitchen)

    (:objects 
        sugar - box
        spam - box
        burner - storage
        countertop - storage
        drawer - storage
        cabinet - storage
    )

    (:init
        (placed sugar burner)
        (placed spam countertop)
        (not_placed_any drawer)
        (not_placed_any cabinet)
        (not_grasping_any)
        (closed drawer)
        (closed cabinet)

        (openable drawer)
        (openable cabinet)
    )

    (:goal (and
        (placed sugar countertop)
        (placed spam drawer)
        (closed drawer)
        (closed cabinet)
    ))
)
