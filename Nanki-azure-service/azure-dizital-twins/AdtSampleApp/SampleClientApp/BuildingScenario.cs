using System.Threading.Tasks;

namespace SampleClientApp
{
    public class BuildingScenario
    {
        private readonly CommandLoop cl;
        public BuildingScenario(CommandLoop cl)
        {
            this.cl = cl;
        }

        public async Task InitBuilding()
        {
            Log.Alert($"Deleting all twins...");
            await cl.DeleteAllTwinsAsync();
            await InitializeGraph();
        }
        private async Task InitializeGraph()
        {
            string[] modelsToUpload = new string[3] {"CreateModels", "Mover", "NankiModel" };
            Log.Out($"Uploading {string.Join(", ", modelsToUpload)} models");

            await cl.CommandCreateModels(modelsToUpload);

            Log.Out($"Creating SpaceModel and Thermostat...");
            await cl.CommandCreateDigitalTwin(new string[3]
                {
                    "CreateTwin", "dtmi:com:nec:example:Nanki;2", "南紀白浜空港",
                });
            await cl.CommandCreateDigitalTwin(new string[3]
                {
                    "CreateTwin", "dtmi:com:nec:example:Nanki;2", "1階フロア",
                });
            await cl.CommandCreateDigitalTwin(new string[3]
                {
                    "CreateTwin", "dtmi:com:nec:example:Nanki;2", "2階フロア",
                });
            await cl.CommandCreateDigitalTwin(new string[3]
                {
                    "CreateTwin", "dtmi:com:nec:example:Nanki;2", "ゾーンA",
                });
            await cl.CommandCreateDigitalTwin(new string[3]
                {
                    "CreateTwin", "dtmi:com:nec:example:Nanki;2", "ゾーンB",
                });
            await cl.CommandCreateDigitalTwin(new string[45]
                {
                    "CreateTwin", "dtmi:com:nec:example:Robot;1", "001",
                    "RobotID", "double", "1",
                    "Name", "string", "thk002",
                    "Vendor", "string", "THK",
                    "ProductNumber", "string", "THK Mover",
                    "ProductType", "string", "Sinage",
                    "PositionX", "double", "2",
                    "PositionY", "double", "1",
                    "PositionZ", "double", "12",
                    "PositionDegree", "double", "12.34",
                    "Status", "string", "advertising",
                    "Battery", "double", "100",
                    "Obstacle_flg", "int","1",
                    "Floor", "string", "1F",
                    "Eroor", "double", "1",

                });
            await cl.CommandCreateDigitalTwin(new string[45]
                {
                    "CreateTwin", "dtmi:com:nec:example:Robot;1", "002",
                    "RobotID", "double", "1",
                    "Name", "string", "thk002",
                    "Vendor", "string", "THK",
                    "ProductNumber", "string", "THK Mover",
                    "ProductType", "string", "Sinage",
                    "PositionX", "double", "2",
                    "PositionY", "double", "4",
                    "PositionZ", "double", "12",
                    "PositionDegree", "double", "12.34",
                    "Status", "string", "advertising",
                    "Battery", "double", "100",
                    "Obstacle_flg", "double","1",
                    "Floor", "string", "1F",
                    "Eroor", "double", "1",

                });

            Log.Out($"Creating edges between the Floor, Room and Thermostat");
            await cl.CommandCreateRelationship(new string[5]
                {
                    "CreateEdge", "南紀白浜空港", "contains", "1階フロア", "floor_to_room_edge",
                });
            await cl.CommandCreateRelationship(new string[5]
                 {
                    "CreateEdge", "南紀白浜空港", "contains", "2階フロア", "floor_to_room_edge2",
                 });
            await cl.CommandCreateRelationship(new string[5]
                 {
                    "CreateEdge", "1階フロア", "contains", "ゾーンA", "floor_to_room_edge3",
                 });
            await cl.CommandCreateRelationship(new string[5]
                 {
                    "CreateEdge", "2階フロア", "contains", "ゾーンB", "floor_to_room_edge4",
                 });
            await cl.CommandCreateRelationship(new string[5]
                 {
                    "CreateEdge", "ゾーンA", "contains", "001", "floor_to_room_edge5",
                 });
            await cl.CommandCreateRelationship(new string[5]
                 {
                    "CreateEdge", "ゾーンA", "contains", "002", "floor_to_room_edge6",
                 });

        }
    }
}
