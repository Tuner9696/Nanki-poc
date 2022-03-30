using Azure;
using Azure.Core.Pipeline;
using Azure.DigitalTwins.Core;
using Azure.Identity;
using Microsoft.Azure.EventGrid.Models;
using Microsoft.Azure.WebJobs;
using Microsoft.Azure.WebJobs.Extensions.EventGrid;
using Microsoft.Extensions.Logging;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Net.Http;

namespace SampleFunctionsApp
{
    public class ProcessHubToDTEvents
    {
        //Htppクライアントのセッション接続
        private static readonly HttpClient httpClient = new HttpClient();
        //azure dizital twinsの接続先URL
        private static string adtServiceUrl = Environment.GetEnvironmentVariable("ADT_SERVICE_URL");

        [FunctionName("ProcessHubToDTEvents")]
        public async void Run([EventGridTrigger]EventGridEvent eventGridEvent, ILogger log)
        {
            //Authenticate with Digital Twins
            var credentials = new DefaultAzureCredential();
            DigitalTwinsClient client = new DigitalTwinsClient(
                new Uri(adtServiceUrl), credentials, new DigitalTwinsClientOptions
                { Transport = new HttpClientTransport(httpClient) });

            if (eventGridEvent != null && eventGridEvent.Data != null)
            {
                //ログ出力
                log.LogInformation(eventGridEvent.Data.ToString());

                // Reading deviceId and temperature for IoT Hub JSON
                JObject deviceMessage = (JObject)JsonConvert.DeserializeObject(eventGridEvent.Data.ToString());
                //ロボットID確認
                string Robot_ID = (string)deviceMessage["body"]["Robot_ID"];
                //ロボットIDでデジタルツウィンの更新先を変更
                if   ( Robot_ID == "001" )  
                {
                    var deviceId = Robot_ID;
                    var name = deviceMessage["body"]["Name"];
                    var vendor = deviceMessage["body"]["Vendor"];
                    var product_number = deviceMessage["body"]["Product_Number"];
                    var product_type = deviceMessage["body"]["Product_Type"];
                    var positionX = deviceMessage["body"]["Position_x"];
                    var positionY = deviceMessage["body"]["Position_y"];
                    var positioinZ = deviceMessage["body"]["Position_z"];
                    var positionDegree = deviceMessage["body"]["Position_Degree"];
                    var status = deviceMessage["body"]["Status"];
                    var battery = deviceMessage["body"]["Battery"];
                    var obstacle_flg = deviceMessage["body"]["Obstacle_flg"];
                    var floor = deviceMessage["body"]["Floor"];
                    var eroor = deviceMessage["body"]["Eroor"];


                    var updateTwinData = new JsonPatchDocument();
                    updateTwinData.AppendReplace("/Name", name.Value<string>());
                    updateTwinData.AppendReplace("/Vendor", vendor.Value<string>());
                    updateTwinData.AppendReplace("/ProductNumber", product_number.Value<string>());
                    updateTwinData.AppendReplace("/ProductType", product_type.Value<string>());
                    updateTwinData.AppendReplace("/PositionX", positionX.Value<double>());
                    updateTwinData.AppendReplace("/PositionY", positionY.Value<double>());
                    updateTwinData.AppendReplace("/PositionZ", positioinZ.Value<double>());
                    updateTwinData.AppendReplace("/PositionDegree", positionDegree.Value<double>());
                    updateTwinData.AppendReplace("/Status", status.Value<string>());
                    updateTwinData.AppendReplace("/Battery", battery.Value<double>());
                    updateTwinData.AppendReplace("/Obstacle_flg", obstacle_flg.Value<double>());
                    updateTwinData.AppendReplace("/Floor", floor.Value<string>());
                    updateTwinData.AppendReplace("/Eroor", eroor.Value<double>());
                    await client.UpdateDigitalTwinAsync(deviceId, updateTwinData);

                }

                else if (Robot_ID == "002")
                {
                    var deviceId = Robot_ID;
                    var name = deviceMessage["body"]["Name"];
                    var vendor = deviceMessage["body"]["Vendor"];
                    var product_number = deviceMessage["body"]["Product_Number"];
                    var product_type = deviceMessage["body"]["Product_Type"];
                    var positionX = deviceMessage["body"]["Position_x"];
                    var positionY = deviceMessage["body"]["Position_y"];
                    var positioinZ = deviceMessage["body"]["Position_z"];
                    var positionDegree = deviceMessage["body"]["Position_Degree"];
                    var status = deviceMessage["body"]["Status"];
                    var battery = deviceMessage["body"]["Battery"];
                    var obstacle_flg = deviceMessage["body"]["Obstacle_flg"];
                    var floor = deviceMessage["body"]["Floor"];
                    var eroor = deviceMessage["body"]["Eroor"];


                    var updateTwinData = new JsonPatchDocument();

                    updateTwinData.AppendReplace("/Name", name.Value<string>());
                    updateTwinData.AppendReplace("/Vendor", vendor.Value<string>());
                    updateTwinData.AppendReplace("/ProductNumber", product_number.Value<string>());
                    updateTwinData.AppendReplace("/ProductType", product_type.Value<string>());
                    updateTwinData.AppendReplace("/PositionX", positionX.Value<double>());
                    updateTwinData.AppendReplace("/PositionY", positionY.Value<double>());
                    updateTwinData.AppendReplace("/PositionZ", positioinZ.Value<double>());
                    updateTwinData.AppendReplace("/PositionDegree", positionDegree.Value<double>());
                    updateTwinData.AppendReplace("/Status", status.Value<string>());
                    updateTwinData.AppendReplace("/Battery", battery.Value<double>());
                    updateTwinData.AppendReplace("/Obstacle_flg", obstacle_flg.Value<double>());
                    updateTwinData.AppendReplace("/Floor", floor.Value<string>());
                    updateTwinData.AppendReplace("/Eroor", eroor.Value<double>());

                    await client.UpdateDigitalTwinAsync(deviceId, updateTwinData);

                }

            }
        }
    }
}