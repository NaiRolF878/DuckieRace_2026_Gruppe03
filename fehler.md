oot@duckie3-ThinkPad-L15-Gen-4:~/DuckieRace# launchers/avoid_ducks.sh 
WARNING ⚠️ Unable to automatically guess model task, assuming 'task=detect'. Explicitly define task for your model, i.e. 'task=detect', 'segment', 'classify', 'pose', 'obb' or 'semantic'.
[INFO] [1784743257.723218]: Intrinsics für Duck-Detection erfolgreich geladen.
[INFO] [1784743257.780690]: Duck Detection Node läuft (Entzerrung aktiv...)
Loading /root/DuckieRace/src/packages/avoid_ducks/src/best.onnx for ONNX Runtime inference...
Using ONNX Runtime 1.16.3 with CPUExecutionProvider
[INFO] [1784743261.714577]: Intrinsics geladen.
[INFO] [1784743261.741091]: Homographie geladen.
[INFO] [1784743261.757594]: HSV Config geladen.
[INFO] [1784743261.987133]: Duck Avoidance Node initialisiert.
Traceback (most recent call last):
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 679, in <module>
    node.run()
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 599, in run
    debug_frame = self._draw_debug_overlay(debug_frame)
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 644, in _draw_debug_overlay
    intent_text = self._state_action_text()
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 611, in _state_action_text
    direction_txt = "links" if self.escape_direction == 1.0 else "rechts"
AttributeError: 'DuckAvoidanceNode' object has no attribute 'escape_direction'oot@duckie3-ThinkPad-L15-Gen-4:~/DuckieRace# launchers/avoid_ducks.sh 
WARNING ⚠️ Unable to automatically guess model task, assuming 'task=detect'. Explicitly define task for your model, i.e. 'task=detect', 'segment', 'classify', 'pose', 'obb' or 'semantic'.
[INFO] [1784743257.723218]: Intrinsics für Duck-Detection erfolgreich geladen.
[INFO] [1784743257.780690]: Duck Detection Node läuft (Entzerrung aktiv...)
Loading /root/DuckieRace/src/packages/avoid_ducks/src/best.onnx for ONNX Runtime inference...
Using ONNX Runtime 1.16.3 with CPUExecutionProvider
[INFO] [1784743261.714577]: Intrinsics geladen.
[INFO] [1784743261.741091]: Homographie geladen.
[INFO] [1784743261.757594]: HSV Config geladen.
[INFO] [1784743261.987133]: Duck Avoidance Node initialisiert.
Traceback (most recent call last):
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 679, in <module>
    node.run()
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 599, in run
    debug_frame = self._draw_debug_overlay(debug_frame)
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 644, in _draw_debug_overlay
    intent_text = self._state_action_text()
  File "/root/DuckieRace/src/packages/avoid_ducks/src/duck_avoidance_node.py", line 611, in _state_action_text
    direction_txt = "links" if self.escape_direction == 1.0 else "rechts"
AttributeError: 'DuckAvoidanceNode' object has no attribute 'escape_direction'
