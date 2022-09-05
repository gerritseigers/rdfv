using Microsoft.AspNetCore.Components;
using Newtonsoft.Json;
using Wur.RDFV.Core.Interfaces;
using Wur.RDFV.Core.Services;
using Wur.RDFV.Core.ViewModels;

namespace Wur.RDFV.Blazor.Pages
{
    partial class DeviceSettings
    {
        [Parameter]
        public string deviceId { get; set; }

        [Inject]
        IMessageToDeviceService messageToDeviceService { get; set; }

        [Inject]
        SettingsService settingsService { get; set; }

        [Inject]
        public NavigationManager MyNavigationManager { get; set; } = default!;

        public SettingsData settingsData { get; set; } = null;
        public string message = "Waiting for settings data. This can take a long time";

        protected async override Task OnInitializedAsync()
        {
            settingsService.InputDataReceived += SettingsService_InputDataReceived;

            var message = "{\"Type\":\"Info\"}";
            await messageToDeviceService.sendMessageToDevice(deviceId, message);
        }

        private void SettingsService_InputDataReceived(object? sender, Core.ViewModels.SettingsData settingsRecord)
        {
            if (settingsRecord.DeviceName == deviceId)
            {
                settingsData = new SettingsData
                {
                    DeviceName = settingsRecord.DeviceName,
                    Buffer = settingsRecord.Buffer,
                    Interval = settingsRecord.Interval,
                    TimeStamp = settingsRecord.TimeStamp,
                    Repeats = settingsRecord.Repeats
                };

                InvokeAsync(() => StateHasChanged());
            }
        }

        public async Task SendSettings()
        {
            var settingsModel = new SettingsModel
            {
                Type = "Settings",
                DeviceName = deviceId,
                Buffer = settingsData.Buffer,
                Interval = settingsData.Interval,
                Repeats = settingsData.Repeats
            };

            await messageToDeviceService.sendMessageToDevice(deviceId, JsonConvert.SerializeObject(settingsModel));
            message = "Message is send to Arduino. Please wait for the new settings";
            settingsData = null;
            InvokeAsync(() => StateHasChanged());
        }

        public async Task CancelSettings()
        {
            MyNavigationManager.NavigateTo($"/");
        }
    }
 }
