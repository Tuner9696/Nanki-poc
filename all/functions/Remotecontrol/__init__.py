import logging
import json
from math import degrees

import random
import sys
import json

import azure.functions as func
from azure.iot.hub import IoTHubRegistryManager
from azure.iot.device import Message
from azure.iot.device import IoTHubDeviceClient

MSG_TXT = "{\"service client sent a message\": %.2f}"
CONNECTION_STRING = "HostName=NankiIoTHubPOC.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=xGbHb/BRVFRuQ+uTFGTr8u8YHGC2B3DQnXafalhYl+8="
DEVICE_ID = "Nanki-test-device"
            
def main(req):
    logging.info('Python HTTP trigger function processed a request.')
    registry_manager = IoTHubRegistryManager(CONNECTION_STRING)

    remote_trigger = req.params.get('remote_trigger')
    

    if not remote_trigger:
        try:
            req_body = req.get_json()
        except ValueError:
            pass
        else:
            remote_trigger = req_body.get('remote_trigger')
            logging.info(remote_trigger)
           
            data = MSG_TXT 
            props={}
            
            props.update(messageId = "message_%d" % 3)
            props.update(correlationId = "correlation_%d" % 3)
            props.update(contentType = "application/json")
            props.update(contentEncoding = "utf-8")
            mission_trigger = None
            props.update(mission_trigger = mission_trigger)


            props.update(remote_trigger = remote_trigger)
            registry_manager.send_c2d_message(DEVICE_ID, data, properties=props)
            

    if remote_trigger:
        #iothub_messaging_sample_run()
        return func.HttpResponse(f"Hello, {remote_trigger}. This HTTP Remotecontrol triggered function executed successfully.")
    else:
        return func.HttpResponse(
             "This HTTP triggered function executed successfully. Pass a name in the query string or in the request body for a personalized response.",
             status_code=200
        )


if __name__ == '__main__':
    print ( "Starting the Python IoT Hub C2D Messaging service sample..." )

    main()