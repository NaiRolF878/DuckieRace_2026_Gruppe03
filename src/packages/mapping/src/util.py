#!/usr/bin/env python3

import json
import os
import copy
import rospy
from std_msgs.msg import String


def _load_merged_parameters(config):
    # Liest Parameter aus der JSON. Es gibt keine bot-spezifischen Overrides
    # mehr – alle Bots teilen sich denselben "default"-Block.
    #
    # Neue JSON-Struktur:
    #   config['parameters']['default']  → Parameter für alle Bots
    #
    # Alte JSON-Struktur (ohne 'default'-Key):
    #   config['parameters']             → wird direkt verwendet (Rückwärtskompatibilität)
    parameters = config['parameters']
    if 'default' not in parameters:
        return parameters
    return copy.deepcopy(parameters['default'])


def init_parameters(node_name, callback_update_parameters):
    # Lädt Parameter beim Start und registriert Callback für Live-Updates.
    path = os.path.join(os.path.dirname(__file__), f"../config/{node_name}.json")
    with open(path, 'r') as f:
        config = json.load(f)

    # Callback nur aufrufen, wenn die Nachricht wirklich für diese Node bestimmt
    # ist (msg['node'] == node_name) – sonst würden Parameteränderungen einer
    # anderen Node hier fälschlich übernommen.
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
