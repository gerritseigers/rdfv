﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Wur.RDFV.Core.ViewModels
{
	public class StableData
	{
		public string DeviceName { get; set; }
		public DateTime TimeStamp { get; set; }

		public float? CO2 { get; set; }
		public float? TEMP { get; set; }
		public float? HUM { get; set; }
		public float? NH3 { get; set; }
		public int Sequence { get; set; }
	}
}
