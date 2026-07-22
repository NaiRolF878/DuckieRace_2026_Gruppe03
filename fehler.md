root@duckie3-ThinkPad-L15-Gen-4:~/DuckieRace# launchers/avoid_ducks.sh 
WARNING ⚠️ Unable to automatically guess model task, assuming 'task=detect'. Explicitly define task for your model, i.e. 'task=detect', 'segment', 'classify', 'pose', 'obb' or 'semantic'.
[INFO] [1784742690.278004]: Intrinsics für Duck-Detection erfolgreich geladen.
[INFO] [1784742690.347403]: Duck Detection Node läuft (Entzerrung aktiv...)
Loading /root/DuckieRace/src/packages/avoid_ducks/src/best.onnx for ONNX Runtime inference...
Using ONNX Runtime 1.16.3 with CPUExecutionProvider
[INFO] [1784742693.938800]: Intrinsics geladen.
[INFO] [1784742693.960016]: Homographie geladen.
[INFO] [1784742693.973502]: HSV Config geladen.
Traceback (most recent call last):
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 678, in <module>
    node = DuckAvoidanceNode('duck_avoidance_node')
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 88, in __init__
    util.init_parameters('duck_avoidance_node', self.cbUpdateParameters)
  File "/root/DuckieRace/src/packages/avoid_ducks/src/util.py", line 10, in init_parameters
    with open(path, 'r') as f:
FileNotFoundError: [Errno 2] No such file or directory: '/root/DuckieRace/src/packages/avoid_ducks/src/../config/duck_avoidance_node.json'
