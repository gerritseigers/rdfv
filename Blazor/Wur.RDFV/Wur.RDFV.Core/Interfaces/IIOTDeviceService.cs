using Wur.RDFV.Core.ViewModels;

namespace Wur.RDFV.Core.Interfaces
{
	public interface IIOTDeviceService
	{
		Task<IEnumerable<DeviceRegistration>> GetIotDevices();
		Task<string> GetEndPoint();
	}
}
