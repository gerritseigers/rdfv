using Microsoft.Azure.Devices;
using Microsoft.Extensions.Options;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Wur.RDFV.Core.Config;
using Wur.RDFV.Core.Interfaces;

namespace Wur.RDFV.Core.Services
{
	public class MessageToDeviceService : IMessageToDeviceService
	{
		private string? connectionString;
		private static ServiceClient serviceClient;

		public MessageToDeviceService(IOptionsMonitor<AzureSettings> optionsMonitor)
		{
			var sensorConfig = optionsMonitor.CurrentValue;
			connectionString = sensorConfig.IOTHubConnectionString;

			serviceClient = ServiceClient.CreateFromConnectionString(connectionString);
		}

		public async Task sendMessageToDevice(string device, string message)
		{
			var commandMessage = new Message(Encoding.ASCII.GetBytes(message));
			await serviceClient.SendAsync(device, commandMessage);
		}
	}
}
