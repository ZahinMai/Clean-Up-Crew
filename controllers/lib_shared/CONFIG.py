# =========================================================
# Auction strategy:
# =========================================================
# "sequential"   -> lowest unused task_id first
# "nearest_task" -> task closest to any known collector pos
# "random"       -> random unassigned, uncompleted task
# =========================================================
AUCTION_STRATEGY = "nearest_first"     # "sequential" | "nearest_task" | "load_balanced"
bid_strategy = "plain_distance"      # "plain_distance" | "queue_aware" (unused rn)
TRASH_SPAWN_STRATEGY = "fixed"               # random | fixed
DANGER_ZONE = 0.1      # meters - stop forward motion
CAUTION_ZONE = 0.2     # meters - slow down significantly
SLOW_ZONE = 0.3        # meters - reduce speed
