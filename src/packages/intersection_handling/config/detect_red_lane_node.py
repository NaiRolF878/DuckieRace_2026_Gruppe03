{
  "debug_image_topics": [
    "/debug/red_lane"
  ],

  "parameters": {
    "performance": {
      "frame_skip":  {"default": 3, "min": 1, "max": 10},
      "show_window": {"default": 0, "min": 0, "max": 1}
    },
    "region": {
      "threshold":      {"default": 800, "min": 0,   "max": 10000},
      "detection_zone": {"default": 0.0, "min": 0.0, "max": 1.0},
      "split_lo":       {"default": 0.35, "min": 0.0, "max": 0.5},
      "split_hi":       {"default": 0.65, "min": 0.5, "max": 1.0}
    },
    "red": {
      "hl":  {"default":   0, "min": 0, "max": 179},
      "hh":  {"default":  10, "min": 0, "max": 179},
      "hl2": {"default": 160, "min": 0, "max": 179},
      "hh2": {"default": 179, "min": 0, "max": 179},
      "sl":  {"default": 100, "min": 0, "max": 255},
      "sh":  {"default": 255, "min": 0, "max": 255},
      "vl":  {"default": 100, "min": 0, "max": 255},
      "vh":  {"default": 255, "min": 0, "max": 255}
    }
  }
}
