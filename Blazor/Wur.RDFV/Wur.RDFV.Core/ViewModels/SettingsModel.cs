using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Wur.RDFV.Core.ViewModels
{
    public class SettingsModel
    {
        public string Type { get; set; }
        public string DeviceName { get; set; }
        public int Buffer { get; set; }
        public int Interval { get; set; }
        public int Repeats { get; set; }
    }
}
