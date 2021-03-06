import logging
import json

import random
import sys
import json

import azure.functions as func
from azure.iot.hub import IoTHubRegistryManager
from azure.iot.device import Message
from azure.iot.device import IoTHubDeviceClient

MSG_TXT = "{\"service client sent a message\": %.2f}"
CONNECTION_STRING = "HostName=NS-IoTHub.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=lxoxL9QQEShP37bLfBYO4SW82bcG7mVNtMu+LQ3WZf8="
DEVICE_ID = "NS-MEC"
            
def main(req):
    logging.info('Python HTTP trigger function processed a request.')
    registry_manager = IoTHubRegistryManager(CONNECTION_STRING)
    
    destination_no = req.params.get('destination_no')

    if not (destination_no):
        try:
            req_body = req.get_json()
        except ValueError:
            pass
        else:
            #地点番号を取り出す
            destination_no = req_body.get('destination_no')
            print(type(destination_no))
            print(type(destination_no))
            print(type(destination_no))
            if not (destination_no is None):

                #C2Dメッセージの書き出し
                data = MSG_TXT 
                props={}

        
                props.update(messageId = "message_%d" % 3)
                props.update(correlationId = "correlation_%d" % 3)
                props.update(contentType = "application/json")
                props.update(contentEncoding = "utf-8")
                props.update(destination_no = destination_no)
                registry_manager.send_c2d_message(DEVICE_ID, data, properties=props)
                return func.HttpResponse(
                    f"地点番号, {destination_no}. This HTTP Remotecontrol triggered function executed successfully.",
                    status_code=200
                 )
            else :
                return func.HttpResponse(
             "functions_error1.プロパティ名が不正です。正確なプロパティに変更してください",
              status_code=400
        )

    else:
        return func.HttpResponse(
             "functions_error2.メッセージの値が不正です。正確な値を入れてください",
             status_code=400
        )
    
if __name__ == '__main__':
    print ( "Starting the Python IoT Hub C2D Messaging service sample..." )
    main()
