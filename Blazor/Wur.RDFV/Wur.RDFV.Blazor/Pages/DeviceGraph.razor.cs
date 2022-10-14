using Microsoft.AspNetCore.Components;
using Radzen.Blazor;
using Wur.RDFV.Core.Services;
using Wur.RDFV.Core.ViewModels;
using Azure.Storage.Files.DataLake;
using Microsoft.Extensions.Options;
using Wur.RDFV.Core.Config;
using Azure;
using Azure.Storage.Files.DataLake.Models;
using Azure.Storage;
using System.IO;
using System.Text;

namespace Wur.RDFV.Blazor.Pages
{
	partial class DeviceGraph
	{

		[Inject]
		IOptionsMonitor<AzureSettings> optionsMonitor { get; set; }

		//public DeviceGraph(IOptionsMonitor<AzureSettings> optionsMonitor)
		//{
		//	var sensorConfig = optionsMonitor.CurrentValue;
		//	datalakeConnectionString = sensorConfig.DatalakeConnectionString;
		//}

		[Parameter]
		public string deviceId { get; set; }


		DateTime? value = DateTime.Now;
		DateTime[] datum = new DateTime[1000];
		int[] CO2 = new int[1000];		
		int[] HUM =new int [1000];
		public List<StableData> items { get; set; } = new List<StableData>();

		public string datalakeConnectionString = "DefaultEndpointsProtocol=https;AccountName=euwstoragerdfvp;AccountKey=STtBvcqG0/y7Q/chtibsgrXsYqxuTfm5MhrxPbxvfb9cqhZuuMfeN6fXrSh1gzZomFtXq0SHoNPu+ASt0Dfszg==;EndpointSuffix=core.windows.net";

		public async Task OnChange(DateTime? value, string name, string format)
		{
			await ReadDataFile(deviceId, (DateTime) value);

			var a = items;
					
			InvokeAsync(() => StateHasChanged());
		}

		protected async override Task OnInitializedAsync()
		{
			 await ReadDataFile(deviceId, System.DateTime.Now);
		}

		public async Task ReadDataFile(string deviceId, DateTime selectedDate)
		{
			var dir = $"{selectedDate.Year}/{deviceId}/{selectedDate.Month}";
			var fileName = $"{selectedDate.Day}.csv";

			var dataLakeClient = new DataLakeServiceClient(datalakeConnectionString);
			var fileSystemClient = dataLakeClient.GetFileSystemClient("data");
			DataLakeDirectoryClient directory = fileSystemClient.GetDirectoryClient(dir);
			var fileSystem = directory.GetFileClient(fileName);


			if (fileSystem.Exists())
			{
				Response<FileDownloadInfo> downloadResponse = await fileSystem.ReadAsync();
				BinaryReader reader = new BinaryReader(downloadResponse.Value.Content);

				String[] lines;
				String[] data;
				byte[] a;
				const int bufferSize = 4096;
				using (var ms = new MemoryStream())
				{
					byte[] buffer = new byte[bufferSize];
					int count;
					while ((count = reader.Read(buffer, 0, buffer.Length)) != 0)
						ms.Write(buffer, 0, count);

					var f = Encoding.ASCII.GetString(ms.ToArray());

					lines = f.Split('\n');
					var counter = 0;
					foreach (var line in lines)
					{
						if (counter != 0)
						{
							data = line.Split(',');
							if (data.Length > 8)
							{
								items.Add(new StableData
								{
									Sequence = counter,
									//TimeStamp = System.Convert.ToDateTime(data[0]),
									CO2 = System.Convert.ToInt16(data[1]),
									HUM = System.Convert.ToInt16(data[2]),
									NH3 = System.Convert.ToInt16(data[3]),
									TEMP = System.Convert.ToInt16(data[4]),
									P1 = System.Convert.ToInt16(data[5]),
									P5 = System.Convert.ToInt16(data[6]),
									P7 = System.Convert.ToInt16(data[7]),
									P8 = System.Convert.ToInt16(data[8]),
									RC = System.Convert.ToInt16(data[9]),
									SS = System.Convert.ToInt16(data[10])
								});
							}
						}
						counter++;
					}
				}
			}
		}
	}
}

