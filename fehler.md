root@duckie3-ThinkPad-L15-Gen-4:~/DuckieRace# ls -la src/packages/mapping/src/detect_lane_node.py src/packages/follow_lane/src/detect_lane_node.py
-rwxr-xr-x 1 1000 1000 24467 Jun 10 20:42 src/packages/follow_lane/src/detect_lane_node.py
-rwxr-xr-x 1 1000 1000 26962 Jul 21 10:37 src/packages/mapping/src/detect_lane_node.py

root@duckie3-ThinkPad-L15-Gen-4:~/DuckieRace# ls -la devel/lib/mapping/detect_lane_node.py
-rwxr-xr-x 1 root root 558 Jul 21 10:22 devel/lib/mapping/detect_lane_node.py

root@duckie3-ThinkPad-L15-Gen-4:~/DuckieRace# head -20 devel/lib/mapping/detect_lane_node.py
#!/usr/bin/python3
# -*- coding: utf-8 -*-
# generated from catkin/cmake/template/script.py.in
# creates a relay to a python script source file, acting as that file.
# The purpose is that of a symlink
python_script = '/root/DuckieRace/src/packages/mapping/src/detect_lane_node.py'
with open(python_script, 'r') as fh:
    context = {
        '__builtins__': __builtins__,
        '__doc__': None,
        '__file__': python_script,
        '__name__': __name__,
        '__package__': None,
    }
    exec(compile(fh.read(), python_script, 'exec'), context)
