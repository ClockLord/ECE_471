""" Demonstration of MQTT protocol - post user data """
# http://www.steves-internet-guide.com/into-mqtt-python-client/

    # setup Mosquitto
    # on Python3:
    # py -m pip install paho-mqtt

    # replace the instructor's BUnetID with yours in the code below

def mqtt_client(server):
    import time as utime

    from paho.mqtt.client import Client as MQTTClient
    mqttc = MQTTClient(b'st32f_emulator', server)
    mqttc.connect(server)
    mqttc.loop_start()
    while True:
        temp=input("T= ")
        datastr    = 'Nate-TSET=%s' % ( str(temp) )
        mqttc.publish('test/bradley_edu', datastr)
        print(datastr)
    mqttc.loop_stop()

# this will run main() if this code is pasted directly into Python console
if __name__ == "__main__":
    mqtt_client('localhost')
