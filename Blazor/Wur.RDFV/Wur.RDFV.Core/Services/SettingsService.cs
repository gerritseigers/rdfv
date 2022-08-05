
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Wur.RDFV.Core.ViewModels;

namespace Wur.RDFV.Core.Services
{
	public class SettingsService
	{
		public event EventHandler<SettingsData> InputDataReceived;

		private async Task OnInputMessageReceived(SettingsData data)
		{
			await Task.Run(() => { InputDataReceived?.Invoke(this, data); });
		}

		public async Task SendData(SettingsData data)
		{
			if (data!=null)
			{
				await OnInputMessageReceived((SettingsData)data);
			}
		}
	}
}
