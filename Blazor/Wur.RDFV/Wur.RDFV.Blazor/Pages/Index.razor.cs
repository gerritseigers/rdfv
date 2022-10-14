using Microsoft.AspNetCore.Components;
using Radzen;
using Wur.RDFV.Core.Interfaces;
using Wur.RDFV.Core.ViewModels;

namespace Wur.RDFV.Blazor.Pages
{
	public partial class Index
	{
		[Inject]
		IIOTDeviceService iOTDeviceService { get; set; }

		[Inject]
		public NavigationManager MyNavigationManager { get; set; } = default!;

		public IEnumerable<DeviceRegistration> devices { get; set; }

		protected async override Task OnInitializedAsync()
		{
			devices = await iOTDeviceService.GetIotDevices();
		}

		void OpenSettings(string deviceId)
		{
			MyNavigationManager.NavigateTo($"/DeviceSettings/{deviceId}");
		}

		void OpenData(string deviceId)
		{
			MyNavigationManager.NavigateTo($"/DeviceData/{deviceId}");
		}

		void OpenGraph(string deviceId)
		{
			MyNavigationManager.NavigateTo($"/DeviceGraph/{deviceId}");
		}
	}
}

