#!/usr/bin/env python3

import json
import os
import tkinter as tk
import cv2
import rospy
import util
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import CompressedImage

class ConfigurationNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Fahrzeugnamen aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']
        
        # Pfad zum config-Ordner, in dem alle JSON-Konfigurationsdateien liegen
        self.config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config'))

        # Topic über das aktualisierte Parameter an alle Nodes gesendet werden
        self.update_topic = f'/{self._vehicle_name}/update_parameters'
        self.publisher = rospy.Publisher(self.update_topic, String, queue_size=1)

        # Subscriber für das aktuell ausgewählte Debug-Bild-Topic (wird dynamisch gesetzt)
        self.image_subscriber = None
        self.image = None

        # Alle verfügbaren Nodes ermitteln: jede JSON-Datei im config-Ordner entspricht einer Node.
        # Dateien ohne "parameters"-Block (z.B. mapping_node.json - wird von graph_state_node
        # direkt geladen, nicht ueber util.init_parameters) sind hier nicht konfigurierbar.
        self.available_nodes = []
        for file_name in sorted(os.listdir(self.config_dir)):
            if file_name.endswith('.json'):
                node_name = os.path.splitext(file_name)[0].replace('.json', '')
                with open(os.path.join(self.config_dir, file_name), 'r') as f:
                    if 'parameters' not in json.load(f):
                        continue
                self.available_nodes.append(node_name)

        # --- tkinter GUI aufbauen ---
        self.root = tk.Tk()
        self.root.title('Duckie Configuration')
        self.root.geometry('860x720')
        # Beim Schließen des Fensters sauber herunterfahren
        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)

        # Auswahl der aktiven Node (Dropdown)
        self.selected_node = tk.StringVar(self.root, value=self.available_nodes[0])
        # Auswahl der aktiven Parametergruppe innerhalb der Node (Dropdown)
        self.selected_group = tk.StringVar(self.root)

        # Node-Dropdown
        tk.Label(self.root, text='Node').pack(anchor='w', padx=10, pady=(10, 0))
        tk.OptionMenu(self.root, self.selected_node, *self.available_nodes, command=self.change_node).pack(fill='x', padx=10)

        # Gruppen-Dropdown (wird dynamisch mit den Gruppen der gewählten Node befüllt)
        tk.Label(self.root, text='Group').pack(anchor='w', padx=10, pady=(10, 0))
        self.group_dropdown = tk.OptionMenu(self.root, self.selected_group, '')
        self.group_dropdown.pack(fill='x', padx=10)

        # Debug-Bild-Dropdown (zeigt die in der JSON definierten debug_image_topics)
        self.image_var = tk.StringVar()
        tk.Label(self.root, text='Debug Image').pack(anchor='w', padx=10, pady=(10, 0))
        self.image_dropown = tk.OptionMenu(self.root, self.image_var, '')
        self.image_dropown.pack(fill='x', padx=10, pady=(10, 0))

        # Scrollbarer Bereich fuer die dynamisch erzeugten Slider - noetig seit
        # turn_segments pro Segment/Feld einen eigenen Slider bekommt (z.B.
        # 3 Richtungen x bis zu 3 Segmente x 3 Felder = deutlich mehr als in
        # die feste Fensterhoehe passt).
        slider_container = tk.Frame(self.root)
        slider_container.pack(fill='both', expand=True, padx=10, pady=10)
        slider_canvas = tk.Canvas(slider_container, highlightthickness=0)
        slider_scrollbar = tk.Scrollbar(slider_container, orient='vertical',
                                         command=slider_canvas.yview)
        self.slider_frame = tk.Frame(slider_canvas)
        self.slider_frame.bind(
            '<Configure>',
            lambda e: slider_canvas.configure(scrollregion=slider_canvas.bbox('all')))
        slider_canvas.create_window((0, 0), window=self.slider_frame, anchor='nw')
        slider_canvas.configure(yscrollcommand=slider_scrollbar.set)
        slider_canvas.pack(side='left', fill='both', expand=True)
        slider_scrollbar.pack(side='right', fill='y')
        slider_canvas.bind_all(
            '<MouseWheel>', lambda e: slider_canvas.yview_scroll(int(-1 * (e.delta / 120)), 'units'))

        # GUI mit der ersten verfügbaren Node initialisieren
        self.change_node(self.selected_node.get())


    def select_group(self, group_name):
        # Ausgewählte Gruppe setzen und Schieberegler neu aufbauen
        self.selected_group.set(group_name)
        self.change_group(group_name)


    def select_image_topic(self, topic_name):
        print(f'changing image topic to {topic_name}')
        self.image_var.set(topic_name)

        # Alten Subscriber abmelden, bevor ein neuer gesetzt wird
        if self.image_subscriber:
            self.image_subscriber.unregister()

        # Sicherstellen, dass das Topic den vollen Pfad mit Fahrzeugnamen enthält
        topic = topic_name if topic_name.startswith(f'/{self._vehicle_name}/') else f'/{self._vehicle_name}{topic_name}'
        
        print(f'changing image topic to {topic}')
        # Neuen Subscriber für das gewählte Debug-Bild-Topic anlegen
        self.image_subscriber = rospy.Subscriber(topic, CompressedImage, self.update_image, queue_size=1)


    def rebuild_group_menu(self):
        # Gruppen-Dropdown mit den Parametergruppen der aktuell gewählten Node befüllen
        groups = list(self.parameters.keys())
        menu = self.group_dropdown['menu']
        menu.delete(0, 'end')
        for group in groups:
            menu.add_command(label=group, command=lambda value=group: self.select_group(value))

        # Debug-Bild-Dropdown mit den Topics aus der JSON-Konfiguration befüllen
        image_menu = self.image_dropown['menu']
        image_menu.delete(0, 'end')
        for topic in util.get_image_topics(self.selected_node.get()):
            image_menu.add_command(label=topic, command=lambda value=topic: self.select_image_topic(value))

        # Erste Gruppe automatisch auswählen und Schieberegler aufbauen
        self.select_group(groups[0] if groups else '')
        self.rebuild_sliders()


    # Slider-Grenzen fuer turn_segments-Felder - dort steht kein {min,max}
    # in der JSON (nur v/omega/duration je Segment), daher fest hier hinterlegt.
    SEGMENT_FIELD_RANGES = {
        'v':        (0.0, 1.0),
        'omega':    (-5.0, 5.0),
        'duration': (0.0, 5.0),
    }

    def rebuild_sliders(self):
        # Alle bestehenden Schieberegler entfernen
        for widget in self.slider_frame.winfo_children():
            widget.destroy()
        self.sliders = {}

        group_data = self.parameters.get(self.selected_group.get(), {})

        # turn_segments-Gruppen (z.B. control_intersection_node): jede
        # Richtung ist eine LISTE von Segment-Objekten ({v, omega, duration})
        # statt eines einzelnen {min,max,default}-Werts - eigener Aufbau mit
        # einem Slider pro Segment/Feld statt der generischen Schleife unten.
        if group_data and all(isinstance(v, list) for v in group_data.values()):
            self._build_segment_sliders(group_data)
            return

        # Für jeden Parameter der aktuell gewählten Gruppe einen Schieberegler erzeugen.
        # Werte ohne {min,max,default} (unbekanntes Format) werden übersprungen
        # statt die GUI mit einem KeyError/TypeError abstürzen zu lassen.
        for name, values in group_data.items():
            if not (isinstance(values, dict) and 'min' in values and 'max' in values and 'default' in values):
                continue

            # Typ ermitteln: float-Schieberegler mit Auflösung 0.01, int-Schieberegler mit 1
            is_float = isinstance(values['min'], float)

            slider = tk.Scale(
                self.slider_frame,
                from_=values['min'],
                to=values['max'],
                orient='horizontal',
                label=name,
                # Jede Änderung am Schieberegler ruft update_parameter auf
                command=lambda value, param=name: self.update_parameter(param, value),
                resolution=0.01 if is_float else 1
            )
            # Schieberegler auf den aktuellen default-Wert setzen
            slider.set(values['default'])
            slider.pack(fill='x', pady=4)
            self.sliders[name] = slider

    def _build_segment_sliders(self, group_data):
        # Ein Slider pro (Richtung, Segment-Index, Feld) - z.B. fuer
        # control_intersection_node.json's turn_segments.{left,right,straight}.
        for direction in sorted(group_data.keys()):
            segments = group_data[direction]
            if not isinstance(segments, list) or not segments:
                continue
            tk.Label(self.slider_frame, text=direction, anchor='w',
                     font=('TkDefaultFont', 9, 'bold')).pack(fill='x', pady=(10, 0))
            for seg_index, segment in enumerate(segments):
                if not isinstance(segment, dict):
                    continue
                for field in ('v', 'omega', 'duration'):
                    if field not in segment:
                        continue
                    lo, hi = self.SEGMENT_FIELD_RANGES[field]
                    slider = tk.Scale(
                        self.slider_frame,
                        from_=lo, to=hi,
                        orient='horizontal',
                        label=f"  Segment {seg_index + 1} · {field}",
                        # Default-Breite (100px) schneidet den zusammengesetzten
                        # Label-Text ab ("omega" -> "ome") - explizit verbreitern.
                        length=280,
                        command=lambda value, d=direction, idx=seg_index, f=field:
                            self.update_segment_parameter(d, idx, f, value),
                        resolution=0.01
                    )
                    slider.set(segment[field])
                    slider.pack(fill='x', pady=2)
                    self.sliders[f"{direction}[{seg_index}].{field}"] = slider


    def change_node(self, *_):
        # Beim Wechsel der Node: Parameter aus der zugehörigen JSON-Datei laden
        # und Gruppen-Dropdown sowie Schieberegler neu aufbauen
        self.parameters = util.load_parameters(self.selected_node.get())
        self.rebuild_group_menu()


    def change_group(self, *_):
        # Beim Wechsel der Gruppe: nur die Schieberegler neu aufbauen
        self.rebuild_sliders()


    def update_image(self, msg):
        # Empfangenes komprimiertes Debug-Bild dekodieren und in einem OpenCV-Fenster anzeigen
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow('Debug Image', cv_image)
        cv2.waitKey(1)


    def update_parameter(self, param, value):
        # Geänderten Parameterwert im lokalen Dictionary speichern
        is_float = isinstance(self.parameters[self.selected_group.get()][param]['min'], float)
        self.parameters[self.selected_group.get()][param]['default'] = float(value) if is_float else int(float(value))

        print(f"Updated {param} to {value} in group {self.selected_group.get()} for node {self.selected_node.get()}")
        self._publish_and_save()

    def update_segment_parameter(self, direction, seg_index, field, value):
        # Geaenderten Wert eines einzelnen turn_segments-Feldes speichern
        # (self.parameters[group][direction][seg_index] ist ein Dict-Objekt -
        # direkte Mutation reicht, self.parameters muss nicht neu zugewiesen werden).
        group = self.selected_group.get()
        segment = self.parameters[group][direction][seg_index]
        segment[field] = float(value)

        print(f"Updated {direction}[{seg_index}].{field} to {value} in group {group} "
              f"for node {self.selected_node.get()}")
        self._publish_and_save()

    def _publish_and_save(self):
        # Gesamte Parameterliste als JSON über ROS publizieren
        # → alle Nodes die cbUpdateParameters abonniert haben, erhalten die neuen Werte live
        payload = {'node': self.selected_node.get(), 'parameters': self.parameters}
        self.publisher.publish(String(data=json.dumps(payload)))

        # Geänderten Wert in die JSON-Datei zurückschreiben
        # → Parameter bleiben beim nächsten Start des configuration_node erhalten
        self.save_parameters()

    def save_parameters(self):
        # Aktuelle Parameter in die JSON-Datei zurückschreiben – immer in
        # "default", es gibt keine bot-spezifischen Blöcke mehr.
        node_name = self.selected_node.get()
        path      = os.path.join(self.config_dir, f'{node_name}.json')
        try:
            with open(path, 'r') as f:
                config = json.load(f)

            if 'default' in config['parameters']:
                config['parameters']['default'] = self.parameters
            else:
                config['parameters'] = self.parameters
            print(f"Saved parameters to {path}")

            with open(path, 'w') as f:
                json.dump(config, f, indent=4)

        except Exception as e:
            rospy.logwarn(f"Could not save parameters to {path}: {e}")


    def run(self):
        # tkinter Hauptschleife starten (blockierend bis Fenster geschlossen wird)
        self.root.mainloop()


    def shutdown(self):
        # Sauber herunterfahren: Subscriber abmelden, OpenCV-Fenster schließen, GUI beenden
        if self.image_subscriber:
            self.image_subscriber.unregister()
        cv2.destroyAllWindows()
        self.root.destroy()
        #rospy.signal_shutdown('User ended program')


if __name__ == '__main__':
    node = ConfigurationNode('configuration_node')
    node.run()
    #rospy.spin()
