using Azure.Storage.Files.DataLake;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Options;
using Newtonsoft.Json.Linq;
using System.Text;
using Wur.RDFV.Core.Config;
using Wur.RDFV.Core.Services;
using Wur.RDFV.Core.ViewModels;

namespace Wur.RDFV.Blazor.Controllers
{
	[Route("api/[controller]")]
	[ApiController]
	public class DataController : ControllerBase
	{
		private readonly ILogger<DataController> logger;
		private readonly DataService dataService;
		private string? datalakeConnectionString;

		public DataController(ILogger<DataController> _logger, DataService _dataService, IOptionsMonitor<AzureSettings> optionsMonitor)
		{
			logger = _logger;
			dataService = _dataService;
			var sensorConfig = optionsMonitor.CurrentValue;
			datalakeConnectionString = sensorConfig.DatalakeConnectionString;
		}

		[HttpGet]
		public async Task<IActionResult> Get()
		{
			return Ok("Alles OK ");
		}

		[HttpPost]
		public async Task<IActionResult> Post()
		{
			logger.LogInformation("Data received");
			try
			{
				using (StreamReader reader = new StreamReader(Request.Body))
				{
					var deviceData = new StableData();

					var data = await reader.ReadToEndAsync();
					dynamic item = JObject.Parse(data);

					//Get the name of the device
					deviceData.DeviceName = item.DeviceName;

					DateTimeOffset dateTimeOffset = DateTimeOffset.FromUnixTimeSeconds(Convert.ToInt32(item.Timestamp) + 7200);
					deviceData.TimeStamp = dateTimeOffset.DateTime;
					try
					{
						deviceData.CO2 = (float) item.CO2;
					}
					catch (Exception ex) { }

					try
					{
						deviceData.TEMP = (float) item.T;
					}
					catch (Exception ex) { }

					try
					{
						deviceData.HUM = (float) item.H;
					}
					catch (Exception ex) { }

					try
					{
						deviceData.NH3 = (float) item.NH3;
					}
					catch (Exception ex) { }

					try
					{
						deviceData.P1 = (float)item.P1;
					}
					catch (Exception ex) { }

					try
					{
						deviceData.P5 = (float)item.P5;
					}
					catch (Exception ex) { }

					try
					{
						deviceData.P7 = (float)item.P7;
					}
					catch (Exception ex) { }

					try
					{
						deviceData.P8 = (float)item.P8;
					}
					catch (Exception ex) { }

					try
					{
						deviceData.SS = (int)item.SS;
					}
					catch (Exception ex) { }

					try
					{
						deviceData.RC = (int)item.RC;
					}
					catch (Exception ex) { }

					//Store data in Storage
					var dir = $"{deviceData.TimeStamp.Year}/{deviceData.DeviceName}/{deviceData.TimeStamp.Month}";
                    var fileName = $"{deviceData.TimeStamp.Day}.csv";
                    var fileHeading = "TimeStamp, Humidity, Temperature, CO2, NH3, P1, P5, P7, P8, RC, Signal\n";
                    byte[] fileHeadingByteArray = Encoding.UTF8.GetBytes(fileHeading);

                    var sensorData = $" {deviceData.TimeStamp}, {deviceData.HUM}, {deviceData.TEMP}, {deviceData.CO2}, {deviceData.NH3}, {deviceData.P1}, {deviceData.P5}, {deviceData.P7}, {deviceData.P8},{deviceData.RC},{deviceData.SS}\n";
                    byte[] sensorDateByteArray = Encoding.UTF8.GetBytes(sensorData);

                    var dataLakeClient = new DataLakeServiceClient(datalakeConnectionString);
                    var fileSystemClient = dataLakeClient.GetFileSystemClient("data");
                    fileSystemClient.CreateIfNotExists();

                    DataLakeDirectoryClient directory = fileSystemClient.CreateDirectory(dir);
                    directory.CreateIfNotExists();

                    var fileSystem = directory.GetFileClient(fileName);
                    if (fileSystem.Exists() == false)
                    {
                        fileSystem.Create();
                        fileSystem.Append(new MemoryStream(fileHeadingByteArray), 0);
                        fileSystem.Flush(fileHeadingByteArray.Length);
                    }
                    long currentLength = fileSystem.GetProperties().Value.ContentLength;

                    fileSystem.Append(new MemoryStream(sensorDateByteArray), currentLength);
                    fileSystem.Flush(currentLength + sensorDateByteArray.Length);

                    await dataService.SendData(deviceData);
				}
				return Ok();
			}
			catch (Exception ex)
			{
				logger.LogError(ex.Message);
				if (ex.InnerException != null)
				{
					logger.LogError($"Innerexception:{ex.InnerException.Message}");
				}

				return BadRequest(ex.Message);
			}
		}
	}
}
