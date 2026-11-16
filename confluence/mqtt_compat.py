"""paho-mqtt 1.x / 2.x compatibility (Ubuntu 26.04 ships 2.x, older systems 1.x)."""
import paho.mqtt.client as mqtt


def make_client(client_id):
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:   # paho-mqtt < 2.0
        return mqtt.Client(client_id=client_id)
