
using Microsoft.Azure.Devices;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Wur.RDFV.Core.Config;
using Wur.RDFV.Core.Enums;
using Wur.RDFV.Core.Interfaces;
using Wur.RDFV.Core.ViewModels;

namespace Wur.RDFV.Core.Services
{
	public class IOTDeviceService : IIOTDeviceService
	{
		private string? connectionString;
		private static RegistryManager registryManager;
		private static ILogger logger;


		public IOTDeviceService(
			IOptionsMonitor<AzureSettings> optionsMonitor,
			ILogger<IOTDeviceService> _logger)
		{
			var sensorConfig = optionsMonitor.CurrentValue;
			connectionString = sensorConfig.IOTHubConnectionString;
			registryManager = RegistryManager.CreateFromConnectionString(connectionString);
			logger = _logger;
		}

		public async Task<IEnumerable<DeviceRegistration>> GetIotDevices()
		{
			logger.LogInformation("Retrieving all IotDevices");
			var IotDeviceList = new List<DeviceRegistration>();
			var query = registryManager.CreateQuery("SELECT * FROM devices", 100);

			try
			{
				while (query.HasMoreResults)
				{
					var page = await query.GetNextAsTwinAsync();
					foreach (var twin in page)
					{

						//Merge hist two together and add it to the list.
						var _device = new DeviceRegistration
						{
							DeviceName = twin.DeviceId,
							IsRegistered = true
						};

						if (twin.ConnectionState == DeviceConnectionState.Connected)
						{
							_device.IsConnected = true;
						}
						IotDeviceList.Add(_device);
					}
				}
				logger.LogInformation($"There are {IotDeviceList.Count} available on the IOT HUB");
				return IotDeviceList.OrderBy(d=>d.DeviceName);
			}
			catch (Exception ex)
			{
				logger.LogError(ex.Message);
				if (ex.InnerException != null)
				{
					logger.LogError($"Inner Exception: {ex.InnerException.Message}");
				}
				return null;
			}
		}


		public async Task<string> GetEndPoint()
		{
			//Returns the endPoint form the IOTHub.
			string[] connectionStringParts = connectionString.Split(';');

			//Find the string poart with HostName= in it. 

			foreach (var connectionStringPart in connectionStringParts)
			{
				if (connectionStringPart.Contains("HostName="))
				{
					return connectionStringPart.Substring(connectionStringPart.IndexOf('=')+1);
				}
			}
			return null;
		}
	}
}
