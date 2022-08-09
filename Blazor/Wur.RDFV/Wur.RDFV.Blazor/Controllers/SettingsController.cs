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
	public class SettingsController : ControllerBase
	{
		private readonly ILogger<SettingsController> logger;
		private readonly SettingsService settingsService;
		private string? datalakeConnectionString;

		public SettingsController(ILogger<SettingsController> _logger, SettingsService _settingsService, IOptionsMonitor<AzureSettings> optionsMonitor)
		{
			logger = _logger;
			settingsService = _settingsService;
			var sensorConfig = optionsMonitor.CurrentValue;
			datalakeConnectionString = sensorConfig.DatalakeConnectionString;
		}



		[HttpPost]
		public async Task<IActionResult> Post()
		{
			logger.LogInformation("Data received");
			using (StreamReader reader = new StreamReader(Request.Body))
			{
				var deviceData = new SettingsData();

				var data = await reader.ReadToEndAsync();
				dynamic item = JObject.Parse(data);

				//Get the name of the device
				deviceData.DeviceName = item.DeviceName;

				DateTimeOffset dateTimeOffset = DateTimeOffset.FromUnixTimeSeconds(Convert.ToInt32(item.Timestamp) + 7200);
				deviceData.TimeStamp = dateTimeOffset.DateTime;
				try
				{
					deviceData.Buffer = item.Buffer;
				}
				catch (Exception ex) { }

				try
				{
					deviceData.Interval = item.Interval;
				}
				catch (Exception ex) { }

				try
				{
					deviceData.Repeats = item.Repeats;
				}
				catch (Exception ex) { }

				try
				{
					deviceData.LastResetCause = item.LastResetCause;
				}
				catch (Exception ex) { }

				try
				{
					deviceData.ICCID = item.ICCID;
				}
				catch (Exception ex) { }

				var dir = $"{deviceData.DeviceName}";
				var fileName = $"{deviceData.TimeStamp.Day}-{deviceData.TimeStamp.Month}-{deviceData.TimeStamp.Year} {deviceData.TimeStamp.Hour}:{deviceData.TimeStamp.Minute}.csv";
				var fileHeading = "ICCID, LastResetCause, Buffer, Interval\n";
				byte[] fileHeadingByteArray = Encoding.UTF8.GetBytes(fileHeading);

				var sensorData = $"{deviceData.ICCID},{deviceData.LastResetCause},{deviceData.Buffer},{deviceData.Interval}\n";
				byte[] sensorDateByteArray = Encoding.UTF8.GetBytes(sensorData);

				var dataLakeClient = new DataLakeServiceClient(datalakeConnectionString);
				var fileSystemClient = dataLakeClient.GetFileSystemClient("settings");
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

				await settingsService.SendData(deviceData);
			}
			return Ok();
		}
	}
}
