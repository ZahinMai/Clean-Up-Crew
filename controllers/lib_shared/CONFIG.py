# =========================================================
# Auction strategy:
# =========================================================
# "sequential"   -> lowest unused task_id first
# "nearest_task" -> task closest to any known collector pos        <- (5 samples taken already - do others)
# "random"       -> random unassigned, uncompleted task
# =========================================================
auction_strategy = "nearest_task"    # "sequential" | "nearest_task" | "load_balanced"
bid_strategy = "plain_distance"      # "plain_distance" | "queue_aware"
trash_spawn_strategy = "random"               # random | deterministic
