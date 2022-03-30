using Azure.Messaging.EventHubs.Consumer;
using Microsoft.Azure.Devices;
using Microsoft.Azure.Devices.Client;
using Microsoft.Azure.Devices.Common.Exceptions;
using System;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace DeviceSimulator
{
    public static class AzureIoTHub
    {
        /// <summary>
        /// Please replace with correct connection string value
        /// The connection string could be got from Azure IoT Hub -> Shared access policies -> iothubowner -> Connection String:
        /// </summary>
        
        /// private const string iotHubConnectionString = "HostName=iot-nec-dev.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=ZL3yUr0Jqt7Rr0oLMclfSwPHU8nBGzw0A4zXcz0O50w=";
            private const string iotHubConnectionString = "HostName=NS-IoTHub.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=lxoxL9QQEShP37bLfBYO4SW82bcG7mVNtMu+LQ3WZf8=";
        // private const string iotHubConnectionString = "HostName=test-kuro.azure-devices.net;SharedAccessKeyName=iothubowner;SharedAccessKey=Pxep+xWrmvSFyzOMaIwtZAxYz0dpw7O7pzmciXQg9o8=";

        /// <summary>
        /// Please replace with correct device connection string
        /// The device connect string could be got from Azure IoT Hub -> Devices -> {your device name } -> Connection string
        /// </summary>
         ///private const string deviceConnectionString = "HostName=iot-nec-dev.azure-devices.net;DeviceId=thermostat67;SharedAccessKey=bxzgpRyiN53aTFrF2kzrNEh+pL1Ncx0ajsxIx9d2YzQ=";
           private const string deviceConnectionString = "HostName=NS-IoTHub.azure-devices.net;DeviceId=nanki-device;SharedAccessKey=bdzB2Zk+ivHg7ZlAkeBihv/N43OSE4g7fK342jU18eU=";
        // private const string deviceConnectionString = "HostName=test-kuro.azure-devices.net;DeviceId=adx-test;SharedAccessKey=IZfLu7DRbUZIp8SU62jZgi6CgX9SxOqpVBA6kMK3UkU=";

        public static async Task<string> CreateDeviceIdentityAsync(string deviceName)
        {
            var registryManager = RegistryManager.CreateFromConnectionString(iotHubConnectionString);
            var device = new Device(deviceName);
            try
            {
                device = await registryManager.AddDeviceAsync(device);
            }
            catch (DeviceAlreadyExistsException)
            {
                device = await registryManager.GetDeviceAsync(deviceName);
            }

            return device.Authentication.SymmetricKey.PrimaryKey;
        }
        
        public static async Task SendDeviceToCloudMessageAsync(CancellationToken cancelToken)
        {

            var deviceClient = DeviceClient.CreateFromConnectionString(deviceConnectionString);


            var rand = new Random();

            while (!cancelToken.IsCancellationRequested)
            {

                
                double minTemperature = 20;
                double minHumidity = 60;
                string Robot_ID = "001";
                string Name = "thk001";
                string Vendor = "THK";
                string Product_Number = "THK Mover";
                string Product_Type  = "test";
                var Position_x = rand.NextDouble() * 15;
                var Position_y = 4.54;
                var Position_z =4.54 ;
                var Position_Degree = 4.54;
                string Status = "adver";
                var Battery =33 ;
                var Obstacle_flg = 5;
                string Floor = "9F";
                double Eroor = rand.NextDouble() * 15;
                double currentTemperature = minTemperature + rand.NextDouble() * 15;
                double currentHumidity = minHumidity + rand.NextDouble() * 20;
               


                // Create JSON message

                string messageBody = JsonSerializer.Serialize(
                    new
                    {
                        //temperature = currentTemperature,
                        //humidity = currentHumidity,
                         Robot_ID = Robot_ID,
                         Name =Name,
                         Vendor =Vendor,
                         Product_Number = Product_Number,
                         Product_Type =Product_Type,
                         Position_x = Position_x,
                         Position_y = Position_y,
                         Position_z = Position_z,
                         Position_Degree = Position_Degree,
                         Status = Status,
                         Battery = Battery,
                         Obstacle_flg = Obstacle_flg,
                         Floor = Floor,
                         Eroor = Eroor,
                         
                    }); ;
 
                var message = new Microsoft.Azure.Devices.Client.Message(Encoding.UTF8.GetBytes(messageBody))
                {
                    ContentType = "application/json",
                    ContentEncoding = "utf-8"
                };
                await deviceClient.SendEventAsync(message);
                Console.WriteLine($"{DateTime.Now} > Sending message: {messageBody}");
                
                //Keep this value above 1000 to keep a safe buffer above the ADT service limits
                //See https://aka.ms/adt-limits for more info
                await Task.Delay(50);
            }
        }





        /*
        public static async Task<string> ReceiveCloudToDeviceMessageAsync()
        {
            var oneSecond = TimeSpan.FromSeconds(1);
            var deviceClient = DeviceClient.CreateFromConnectionString(deviceConnectionString);

            while (true)
            {
                var receivedMessage = await deviceClient.ReceiveAsync();
                if (receivedMessage == null)
                {
                    await Task.Delay(oneSecond);
                    continue;
                }

                var messageData = Encoding.ASCII.GetString(receivedMessage.GetBytes());
                await deviceClient.CompleteAsync(receivedMessage);
                return messageData;
            }
        }

       */


        public static async Task ReceiveMessagesFromDeviceAsync(CancellationToken cancelToken)
        {
            try
            {
                string eventHubConnectionString = await IotHubConnection.GetEventHubsConnectionStringAsync(iotHubConnectionString);
                await using var consumerClient = new EventHubConsumerClient(
                    EventHubConsumerClient.DefaultConsumerGroupName,
                    eventHubConnectionString);

                await foreach (PartitionEvent partitionEvent in consumerClient.ReadEventsAsync(cancelToken))
                {
                    if (partitionEvent.Data == null) continue;

                    string data = Encoding.UTF8.GetString(partitionEvent.Data.Body.ToArray());
                    Console.WriteLine($"Message received. Partition: {partitionEvent.Partition.PartitionId} Data: '{data}'");
                }
            }
            catch (TaskCanceledException) { } // do nothing
            catch (Exception ex)
            {
                Console.WriteLine($"Error reading event: {ex}");
            }
        }
         



    }
}
