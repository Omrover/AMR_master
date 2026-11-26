"""Cliente paho compatible con 1.x y 2.x (mismo enfoque que amr_mqtt_bridge)."""


def make_client(client_id='', will=None, username='', password=''):
    """will = (topic, payload, qos, retain) opcional."""
    import paho.mqtt.client as mqtt

    if hasattr(mqtt, 'CallbackAPIVersion'):
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            client_id=client_id, clean_session=True)
    else:
        client = mqtt.Client(client_id=client_id, clean_session=True)
    client.reconnect_delay_set(min_delay=1, max_delay=5)
    if username:
        client.username_pw_set(username, password or None)
    if will is not None:
        topic, payload, qos, retain = will
        client.will_set(topic, payload, qos=qos, retain=retain)
    return client
