using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Http;
using Microsoft.Azure.EventGrid.Models;
using Microsoft.Azure.WebJobs;
using Microsoft.Azure.WebJobs.Extensions.Http;
using Microsoft.Azure.WebJobs.Extensions.EventGrid;
using Microsoft.Azure.WebJobs.Extensions.SignalRService;
using Microsoft.Extensions.Logging;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace SignalRFunction
{
    public static class SignalRFunctions

    {
        public static string robot_ID;
        public static string name;
        public static string vendor;
        public static string product_number;
        public static string product_type;
        public static double position_x;
        public static double position_y;
        public static double position_z;
        public static double position_degree;
        public static string floor;
        public static string status;
        public static double battery;
        public static double error;
        public static double obstacle_flg;

        [FunctionName("negotiate")]
        public static SignalRConnectionInfo GetSignalRInfo(
            [HttpTrigger(AuthorizationLevel.Anonymous, "post")] HttpRequest req,
            [SignalRConnectionInfo(HubName = "dttelemetry")] SignalRConnectionInfo connectionInfo)
        {
            return connectionInfo;
        }

        [FunctionName("broadcast")]
        public static Task SendMessage(
            [EventGridTrigger] EventGridEvent eventGridEvent,
            [SignalR(HubName = "dttelemetry")] IAsyncCollector<SignalRMessage> signalRMessages,
            ILogger log)
        {
           //Eventgridからデータ取り出し
           JObject eventGridData = (JObject)JsonConvert.DeserializeObject(eventGridEvent.Data.ToString());

           //ログ出力 
           log.LogInformation($"Event grid message: {eventGridData}");

           //以下メッセージの取り出し
           var patch0 = (JObject)eventGridData["data"]["patch"][0];
           var patch1 = (JObject)eventGridData["data"]["patch"][1];
           var patch2 = (JObject)eventGridData["data"]["patch"][2];
           var patch3 = (JObject)eventGridData["data"]["patch"][3];
           var patch4 = (JObject)eventGridData["data"]["patch"][4];
           var patch5 = (JObject)eventGridData["data"]["patch"][5];
           var patch6 = (JObject)eventGridData["data"]["patch"][6];
           var patch7 = (JObject)eventGridData["data"]["patch"][7];
           var patch8 = (JObject)eventGridData["data"]["patch"][8];
           var patch9 = (JObject)eventGridData["data"]["patch"][9];
           var patch10 = (JObject)eventGridData["data"]["patch"][10];
           var patch11 = (JObject)eventGridData["data"]["patch"][11];
           var patch12 = (JObject)eventGridData["data"]["patch"][12];

           name = patch0["value"].ToString();
           vendor = patch1["value"].ToString();
           product_number = patch2["value"].ToString();
           product_type = patch3["value"].ToString();
           position_x = Math.Round(patch4["value"].ToObject<double>(), 2);
           position_y = Math.Round(patch5["value"].ToObject<double>(), 2);
           position_z = Math.Round(patch6["value"].ToObject<double>(), 2);
           position_degree = Math.Round(patch7["value"].ToObject<double>(), 2);
           status = patch8["value"].ToString();
           battery = patch9["value"].ToObject<double>();
           obstacle_flg = patch10["value"].ToObject<double>();
           floor = patch11["value"].ToString();
           error = Math.Round(patch12["value"].ToObject<double>(), 2);

           var message = new Dictionary<object, object>
           {
            { "name", name},
            { "vendor", vendor},
            { "product_number", product_number},
            { "product_type", product_type},
            { "positionX", position_x},
            { "positionY", position_y},
            { "positionZ", position_z},
            { "position_Degree", position_degree},
            { "status", status},
            { "battery", battery},
            { "obstacle_flg", obstacle_flg},
            { "floor", floor},
            { "error", error},
             };

            return signalRMessages.AddAsync(
                new SignalRMessage
                {
                    Target = "newMessage",
                    Arguments = new[] { message }
                });
           
        }
    }
}