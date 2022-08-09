using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Wur.RDFV.Core.ViewModels
{
	public class SettingsData
	{
		public string DeviceName { get; set; }
		public DateTime TimeStamp { get; set; }

		public int Buffer { get; set; }
		public int Interval { get; set; }
		public int Repeats {get; set; }
		public string? LastResetCause { get; set; }
		public string? ICCID {get;set;}
	}
}
