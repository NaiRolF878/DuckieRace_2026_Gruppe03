#!/usr/bin/env python3

import json
import os
import copy
import rospy
from std_msgs.msg import String


def _deep_merge(base, override):
    # Merged zwei Parameter-Dictionaries:
    # Werte aus override überschreiben Werte aus base.
    # Nur Gruppen die in override definiert sind werden überschrieben
    # → nicht genannte Gruppen bleiben unverändert aus base erhalten.
    #
    # Beispiel:
    #   base:     {"pid": {"p": 8.0, "i": 0.0, "d": 6.0}, "stop_line": {...}}
    #   override: {"pid": {"p": 5.0, "d": 4.0}}
    #   result:   {"pid": {"p": 5.0, "i": 0.0, "d": 4.0}, "stop_line": {...}}
    result = copy.deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            # Rekursiv mergen wenn beide Werte Dictionaries sind
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _load_merged_parameters(config):
    # Liest Parameter aus der JSON und merged default mit bot-spezifischen Werten.
    #
    # Neue JSON-Struktur:
    #   config['parameters']['default']       → Parameter für alle Bots
    #   config['parameters'][vehicle_name]    → Überschreibungen für diesen Bot (optional)
    #
    # Alte JSON-Struktur (ohne 'default'-Key):
    #   config['parameters']                  → wird direkt verwendet (Rückwärtskompatibilität)
    vehicle_name = os.environ['VEHICLE_NAME']
    parameters   = config['parameters']

    # Prüfen ob neue Struktur mit 'default'-Key vorliegt
    if 'default' not in parameters:
        # Alte Struktur → direkt zurückgeben (Rückwärtskompatibilität)
        return parameters

    # Neue Struktur: default laden
    merged = copy.deepcopy(parameters['default'])

    # Bot-spezifische Überschreibungen laden und mergen
    if vehicle_name in parameters:
        bot_overrides = parameters[vehicle_name]
        merged = _deep_merge(merged, bot_overrides)
        print(f"[util] Bot-spezifische Parameter für '{vehicle_name}' geladen und gemergt.")
    else:
        print(f"[util] Kein bot-spezifischer Eintrag für '{vehicle_name}' – verwende default.")

    return merged


def init_parameters(node_name, callback_update_parameters):
    # Lädt Parameter beim Start und registriert Callback für Live-Updates.
    path = os.path.join(os.path.dirname(__file__), f"../config/{node_name}.json")
    with open(path, 'r') as f:
        config = json.load(f)

    # Bug-Fix: callback_update_parameters wurde im Original immer aufgerufen,
    # auch wenn die Message für eine andere Node bestimmt war.
    # Korrigiert: Callback nur aufrufen wenn msg['node'] == node_name.
    def callback_wrapper(msg):
        data = json.loads(msg.data)
        if data['node'] == node_name:
            parameters = data['parameters']
            print(f"[util] Neue Parameter für {node_name} empfangen.")
            callback_update_parameters(parameters)

    # Beim Start: gemergete Parameter direkt laden und übergeben
    merged = _load_merged_parameters(config)
    callback_update_parameters(merged)

    vehicle_name = os.environ['VEHICLE_NAME']
    rospy.Subscriber(f'/{vehicle_name}/update_parameters', String, callback_wrapper, queue_size=1)


def load_parameters(node_name):
    # Lädt Parameter für den configuration_node (GUI-Aufbau).
    # Gibt die gemergeten Parameter zurück damit die GUI
    # die bot-spezifischen Startwerte korrekt anzeigt.
    path = os.path.join(os.path.dirname(__file__), f"../config/{node_name}.json")
    with open(path, 'r') as f:
        config = json.load(f)
    return _load_merged_parameters(config)


def get_image_topics(node_name):
    # Liest debug_image_topics aus der JSON für das Bild-Dropdown der GUI.
    path = os.path.join(os.path.dirname(__file__), f"../config/{node_name}.json")
    with open(path, 'r') as f:
        config = json.load(f)
    return config['debug_image_topics']
