{
    "debug_image_topics": [
        "/debug/original",
        "/debug/annotated",
        "/debug/bird_view",
        "/debug/lane_white",
        "/debug/lane_red",
        "/debug/duck_bev",
        "/debug/duck_original"
    ],
    "parameters": {
        "default": {
            "crop_image": {
                "bottom_left_x": {
                    "default": 787,
                    "min": -100,
                    "max": 1000
                },
                "bottom_left_y": {
                    "default": 577,
                    "min": -100,
                    "max": 1000
                },
                "bottom_right_x": {
                    "default": -100,
                    "min": -100,
                    "max": 1000
                },
                "bottom_right_y": {
                    "default": 620,
                    "min": -100,
                    "max": 1000
                },
                "top_left_x": {
                    "default": 231,
                    "min": -100,
                    "max": 1000
                },
                "top_left_y": {
                    "default": 234,
                    "min": -100,
                    "max": 1000
                },
                "top_right_x": {
                    "default": 464,
                    "min": -100,
                    "max": 1000
                },
                "top_right_y": {
                    "default": 230,
                    "min": -100,
                    "max": 1000
                }
            },
            "white": {
                "hl": {
                    "default": 0,
                    "min": 0,
                    "max": 255
                },
                "hh": {
                    "default": 255,
                    "min": 0,
                    "max": 255
                },
                "sl": {
                    "default": 0,
                    "min": 0,
                    "max": 255
                },
                "sh": {
                    "default": 41,
                    "min": 0,
                    "max": 255
                },
                "vl": {
                    "default": 161,
                    "min": 0,
                    "max": 255
                },
                "vh": {
                    "default": 255,
                    "min": 0,
                    "max": 255
                },
                "max_frame_jump": {
                    "default": 80,
                    "min": 0,
                    "max": 400
                }
            },
            "red": {
                "hl": {
                    "default": 0,
                    "min": 0,
                    "max": 179
                },
                "hh": {
                    "default": 10,
                    "min": 0,
                    "max": 179
                },
                "hl2": {
                    "default": 160,
                    "min": 0,
                    "max": 179
                },
                "hh2": {
                    "default": 179,
                    "min": 0,
                    "max": 179
                },
                "sl": {
                    "default": 100,
                    "min": 0,
                    "max": 255
                },
                "sh": {
                    "default": 255,
                    "min": 0,
                    "max": 255
                },
                "vl": {
                    "default": 100,
                    "min": 0,
                    "max": 255
                },
                "vh": {
                    "default": 255,
                    "min": 0,
                    "max": 255
                },
                "pixel_threshold": {
                    "default": 500,
                    "min": 0,
                    "max": 5000
                },
                "detection_zone": {
                    "default": 0.95,
                    "min": 0.0,
                    "max": 1.0
                },
                "detection_x_start": {
                    "default": 0.4,
                    "min": 0.0,
                    "max": 1.0
                },
                "detection_x_end": {
                    "default": 1.0,
                    "min": 0.0,
                    "max": 1.0
                }
            },
            "duck": {
                "roi_top": {
                    "default": 0.35,
                    "min": 0.0,
                    "max": 1.0
                },
                "roi_bottom": {
                    "default": 1.0,
                    "min": 0.0,
                    "max": 1.0
                },
                "min_area": {
                    "default": 25,
                    "min": 0,
                    "max": 20000
                },
                "min_w": {
                    "default": 12,
                    "min": 0,
                    "max": 400
                },
                "min_h": {
                    "default": 12,
                    "min": 0,
                    "max": 400
                },
                "kf_process_var": {
                    "default": 0.01,
                    "min": 0.0,
                    "max": 1.0
                },
                "kf_measurement_var": {
                    "default": 0.05,
                    "min": 0.0,
                    "max": 1.0
                },
                "kf_max_missed_frames": {
                    "default": 5,
                    "min": 0,
                    "max": 30
                }
            },
            "obstacle_color": {
                "yellow_hl": {
                    "default": 10,
                    "min": 0,
                    "max": 179
                },
                "yellow_hh": {
                    "default": 60,
                    "min": 0,
                    "max": 179
                },
                "yellow_sl": {
                    "default": 45,
                    "min": 0,
                    "max": 255
                },
                "yellow_sh": {
                    "default": 255,
                    "min": 0,
                    "max": 255
                },
                "yellow_vl": {
                    "default": 80,
                    "min": 0,
                    "max": 255
                },
                "yellow_vh": {
                    "default": 255,
                    "min": 0,
                    "max": 255
                },
                "green_hl": {
                    "default": 40,
                    "min": 0,
                    "max": 179
                },
                "green_hh": {
                    "default": 85,
                    "min": 0,
                    "max": 179
                },
                "green_sl": {
                    "default": 60,
                    "min": 0,
                    "max": 255
                },
                "green_sh": {
                    "default": 255,
                    "min": 0,
                    "max": 255
                },
                "green_vl": {
                    "default": 40,
                    "min": 0,
                    "max": 255
                },
                "green_vh": {
                    "default": 255,
                    "min": 0,
                    "max": 255
                }
            },
            "white_follow": {
                "offset_px": {
                    "default": 130,
                    "min": 0,
                    "max": 400
                }
            },
            "zones": {
                "corridor_width_px": {
                    "default": 300,
                    "min": 0,
                    "max": 400
                },
                "far_y_min": {
                    "default": 0.25,
                    "min": 0.0,
                    "max": 1.0
                },
                "far_y_max": {
                    "default": 0.45,
                    "min": 0.0,
                    "max": 1.0
                },
                "mid_y_min": {
                    "default": 0.45,
                    "min": 0.0,
                    "max": 1.0
                },
                "mid_y_max": {
                    "default": 0.7,
                    "min": 0.0,
                    "max": 1.0
                },
                "near_y_min": {
                    "default": 0.7,
                    "min": 0.0,
                    "max": 1.0
                },
                "near_y_max": {
                    "default": 0.95,
                    "min": 0.0,
                    "max": 1.0
                },
                "pixel_threshold_frac": {
                    "default": 0.01,
                    "min": 0.0,
                    "max": 1.0
                }
            }
        }
    }
}




root@duckie3-ThinkPad-L15-Gen-4:~/DuckieRace# launchers/mapping.sh 
Traceback (most recent call last):
  File "/root/DuckieRace/devel/lib/mapping/switch_control_node.py", line 15, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/root/DuckieRace/src/packages/mapping/src/switch_control_node.py", line 24, in <module>
    import util
ModuleNotFoundError: No module named 'util'
Traceback (most recent call last):
  File "/root/DuckieRace/devel/lib/mapping/detect_lane_node.py", line 15, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/root/DuckieRace/src/packages/mapping/src/detect_lane_node.py", line 24, in <module>
    import util
ModuleNotFoundError: No module named 'util'
Traceback (most recent call last):
  File "/root/DuckieRace/devel/lib/mapping/detect_apriltag_node.py", line 15, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/root/DuckieRace/src/packages/mapping/src/detect_apriltag_node.py", line 31, in <module>
    import util
ModuleNotFoundError: No module named 'util'
[INFO] [1784220038.451912]: [explore_control_node] Bereit. 5 Kanten zu erkunden.
[INFO] [1784220038.598366]: [graph_state_node] Bereit. Start-Knoten: A
[INFO] [1784220038.661250]: [path_planner_node] Bereit. Delivery-Start: A
[INFO] [1784220038.911279]: [debug_graph_node] Bereit.
Traceback (most recent call last):
  File "/root/DuckieRace/devel/lib/mapping/control_lane_node.py", line 15, in <module>
Traceback (most recent call last):
  File "/root/DuckieRace/devel/lib/mapping/control_intersection_node.py", line 15, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/root/DuckieRace/src/packages/mapping/src/control_lane_node.py", line 16, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/root/DuckieRace/src/packages/mapping/src/control_intersection_node.py", line 30, in <module>
    import util
ModuleNotFoundError: No module named 'util'
    import util
ModuleNotFoundError: No module named 'util'
