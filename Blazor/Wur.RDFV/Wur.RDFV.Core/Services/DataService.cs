
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Wur.RDFV.Core.ViewModels;

namespace Wur.RDFV.Core.Services
{
	public class DataService
	{
		public event EventHandler<StableData> InputDataReceived;

		private async Task OnInputMessageReceived(StableData data)
		{
			await Task.Run(() => { InputDataReceived?.Invoke(this, data); });
		}

		public async Task SendData(StableData data)
		{
			if (data!=null)
			{
				await OnInputMessageReceived((StableData)data);
			}
		}
	}
}
