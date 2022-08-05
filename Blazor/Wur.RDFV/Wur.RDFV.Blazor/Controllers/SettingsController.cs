using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Newtonsoft.Json.Linq;
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
		public SettingsController(ILogger<SettingsController> _logger, SettingsService _settingsService)
		{
			logger = _logger;
			settingsService = _settingsService;
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


				await settingsService.SendData(deviceData);
			}
			return Ok();
		}
	}
}
