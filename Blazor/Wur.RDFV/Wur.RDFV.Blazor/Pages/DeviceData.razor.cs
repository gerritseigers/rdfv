using Microsoft.AspNetCore.Components;
using Radzen.Blazor;
using Wur.RDFV.Core.Services;
using Wur.RDFV.Core.ViewModels;

namespace Wur.RDFV.Blazor.Pages
{
	partial class DeviceData
	{
		[Parameter]
		public string deviceId { get; set; }

		[Inject]
		public DataService dataService { get; set; }
		[Inject]
		public NavigationManager MyNavigationManager { get; set; } = default!;

		public StableData lastValueRead { get; set; } = null;
		RadzenChart humidityChart;
		public List<StableData> items { get; set; } = new List<StableData>();
		public readonly int MaximumNumberOfItems = 100;

		protected async override Task OnInitializedAsync()
		{
			dataService.InputDataReceived += DataService_InputDataReceived;

			//Fill the items list with default values 
			for (int ii = 0; ii < MaximumNumberOfItems; ii++)
			{
				items.Add(new StableData
				{
					Sequence = ii,
					CO2 = 0F,
					HUM = 0F,
					NH3 = 0F,
					TEMP = 0F,
					P1 = 0F,
					P5 = 0F,
					P7 = 0F,
					P8 = 0F,
					SS = 0,
					RC = 0
				});
			}
		}

		public async Task CancelSettings()
		{
			MyNavigationManager.NavigateTo($"/");
		}

		private void AddRecordToItems(StableData dataRecord)
		{
			//Add the record to the end of the items array
			for (int ii = 0; ii < MaximumNumberOfItems-1; ii++)
			{
				var item = items[ii+1];
				item.Sequence = ii;
				items[ii] = item;
			}

			dataRecord.Sequence = MaximumNumberOfItems-1;
			items[MaximumNumberOfItems-1] = dataRecord;
		}

		private void DataService_InputDataReceived(object? sender, Core.ViewModels.StableData dataRecord)
		{
			if (dataRecord.DeviceName == deviceId)
			{
				lastValueRead = new StableData
				{
					DeviceName = dataRecord.DeviceName,
					TimeStamp = dataRecord.TimeStamp,
					CO2 = dataRecord.CO2,
					HUM = dataRecord.HUM,
					NH3 = dataRecord.NH3,
					TEMP = dataRecord.TEMP,
					P1 = dataRecord.P1,
					P5 = dataRecord.P5,
					P7 = dataRecord.P7,
					P8 = dataRecord.P8,
					SS = dataRecord.SS,
					RC = dataRecord.RC,
				};

				AddRecordToItems(lastValueRead);
				items = items.ToList();

				InvokeAsync(() => StateHasChanged());
			}
		}
	}
}
