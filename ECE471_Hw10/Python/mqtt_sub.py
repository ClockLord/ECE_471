""" Demonstration of MQTT protocol - monitor only """
# http://www.steves-internet-guide.com/into-mqtt-python-client/

    # setup Mosquitto
    # from console window run:
    # py -m pip install paho-mqtt

def callback_function_PahoMQTT(client, userdata, message):
    print('Received: ', ( message.topic, str(message.payload.decode("utf-8", errors='replace')) ) )

def mqtt_client(server):
    import time
    machine_id = b'st32f_monitor' 
    # *** IMPORTANT!!! *** There must not be two subscribers with the same ID ***

    from paho.mqtt.client import Client as MQTTClient
    mqttc = MQTTClient(machine_id, server)
    mqttc.connect(server)
    mqttc.on_message = callback_function_PahoMQTT
    mqttc.subscribe('test/#')
    mqttc.subscribe('test/bradley_edu')
    mqttc.loop_start()
    while True:
        time.sleep(1)
    mqttc.loop_stop()
    
# this will run main() if this code is pasted directly into Python console
if __name__ == "__main__":
    mqtt_client('192.168.1.129')