using System;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Wur.RDFV.Core.Enums;

namespace Wur.RDFV.Core.ViewModels
{
	public class DeviceRegistration
	{
		public DeviceRegistration()
		{
			IsConnected = false;
			IsRegistered = false;
			SensorType = null;
		}

		[Key]
		public int Id { get; set; }
		public string? DeviceName { get; set; }
		public string? SensorType { get; set; }
		public string? EndPoint { get; set; }
		public bool IsRegistered { get; set; }
		public bool IsConnected { get; set; }
		public int Buffer { get; set; }
		public int Interval { get; set; }
		public string? IMeiCode { get; set; }
		public string? PinCode { get; set; }
		public string? SHA1 { get; set; }
		public IOTDeviceAuthenticationType? RegistrationType { get; set; }
	}
}
