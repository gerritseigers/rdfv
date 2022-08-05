using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Wur.RDFV.Core.Interfaces
{
	public interface IMessageToDeviceService
	{
		Task sendMessageToDevice(string device, string message);
	}
}
