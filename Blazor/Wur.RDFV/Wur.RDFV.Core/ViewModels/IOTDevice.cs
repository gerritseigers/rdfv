using System;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Wur.RDFV.Core.Enums;

namespace Wur.RDFV.Core.ViewModels
{
	public class IOTDevice
	{
		[Key]
		public int SensorID { get; set; }
		public string SensorName { get; set; }
		public string? Status { get; set; }
		public string? ConnectedState { get; set; }
		public IOTDeviceAuthenticationType? AuthenticationType { get; set; }
		public string? PrimaryThumbprint { get; set; }
		public string? SecondaryThumbprint { get; set; }

	}
}
