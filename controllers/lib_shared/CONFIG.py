# =========================================================
# Auction strategy:
# =========================================================
# "sequential"   -> lowest unused task_id first
# "nearest_task" -> task closest to any known collector pos
# "random"       -> random unassigned, uncompleted task
# =========================================================
AUCTION_STRATEGY = "nearest_task"     # "sequential" | "nearest_task" | "load_balanced"
bid_strategy = "plain_distance"      # "plain_distance" | "queue_aware" (unused rn)
TRASH_SPAWN_STRATEGY = "fixed"               # random | fixed
DANGER_ZONE = 0.1      # meters - stop forward motion
CAUTION_ZONE = 0.2     # meters - slow down significantly
SLOW_ZONE = 0.3        # meters - reduce speed
FIXED_TRASH_LOCATIONS1 = [
    (-1.167, -0.833),
    ( 3.167,  5.167),
    (-3.167,  5.167),
    ( 3.167, -5.167),
    (-3.167, -5.167),
    ( 1.833,  1.500),
    ( 2.500, -1.833),
    (-3.167,  1.833),
    (-0.167, -5.167),
    (-0.167,  5.167),
]

FIXED_TRASH_LOCATIONS = [
    ( 0.500, -1.833),
    (-2.167,  1.833),
    (-2.500, -1.833),
    ( 1.500,  2.167),
    ( 1.833, -1.833),
    (-1.167, -1.833),
    (-1.167,  2.500),
    (-1.167,  1.167),
    (-0.167,  1.833),
    (-1.167, -4.833)
]


FIXED_TRASH_LOCATIONS3 = [
    ( 1.167, -2.500),
    (-1.833,  2.833),
    (-0.500, -3.833),
    ( 2.167,  1.167),
    (-2.167, -2.500),
    ( 0.833,  3.833),
    (-1.167,  0.500),
    ( 1.833, -0.833),
    (-0.167,  4.167),
    ( 0.167, -4.167)
]