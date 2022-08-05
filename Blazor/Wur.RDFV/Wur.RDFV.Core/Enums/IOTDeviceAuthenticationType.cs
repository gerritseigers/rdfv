using System;
using System.Collections.Generic;
using System.ComponentModel.DataAnnotations;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Wur.RDFV.Core.Enums
{
	public enum IOTDeviceAuthenticationType
	{
		[Display(Name = "Symetric")]
		Symetric,
		[Display(Name = "X.509 Self-Signed")]
		X509SelfSigned,
		[Display(Name = "X.509 CA Signed")]
		X509CASigned
	}
}
