# =========================================================
# "sequential"   -> lowest unused task_id first
# "nearest_task" -> task closest to any known collector pos
# "random"       -> random unassigned, uncompleted task
# =========================================================
AUCTION_STRATEGY = "sequential"
bid_strategy = "plain_distance"      # "plain_distance" | "queue_aware" (unused rn)
TRASH_SPAWN_STRATEGY = "fixed"       # random | fixed
DANGER_ZONE = 0.1      # meters - stop forward motion
CAUTION_ZONE = 0.2     # meters - slow down significantly
SLOW_ZONE = 0.3        # meters - reduce speed
TRASH_LAYOUT  = "trash layout 1"

FIXED_TRASH_LOCATIONS = {
    "trash layout 1": {
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
    },

    "trash layout 2": {
        ( 0.500, -1.833),
        (-2.167,  1.833),
        (-2.500, -1.833),
        ( 1.500,  2.167),
        ( 1.833, -1.833),
        (-1.167, -1.833),
        (-1.167,  2.500),
        (-1.167,  1.167),
        (-0.167,  1.833),
        (-1.167, -4.833),
    },

    "trash layout 3": {
        ( 1.167, -2.500),
        (-1.833,  2.833),
        (-0.500, -3.833),
        ( 2.167,  1.167),
        (-2.167, -2.500),
        ( 0.833,  3.833),
        (-1.167,  0.500),
        ( 1.833, -0.833),
        (-0.167,  4.167),
        ( 0.167, -4.167),
    }
}

TRASH_TEMPLATE = """
DEF TRASH_%d Solid {
  translation %f %f 0.1
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0 0 1
        roughness 0.5
        metalness 0
      }
      geometry Sphere {
        radius 0.05
      }
    }
  ]
  boundingObject Sphere {
    radius 0.05
  }
  physics Physics {
    density -1
    mass 1
  }
}
"""
# How long to wait for bids before closing an auction
AUCTION_WAIT_TIME = 2.0  # in seconds