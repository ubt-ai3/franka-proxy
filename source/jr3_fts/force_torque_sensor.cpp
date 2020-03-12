#include "force_torque_sensor.hpp"

#include <vector>
#include <numeric>

#include <Windows.h>

#include "fore_torque_sensor_JR3PCIIoctls.h"


extern "C" 
{


WORD ReadWord(HANDLE hJr3PciDevice, UCHAR ucChannel, ULONG ulOffset)
{
	JR3PCI_READ_WORD_REQUEST_PARAMS ReadWordRequestParams;
	ReadWordRequestParams.ucChannel = ucChannel;
	ReadWordRequestParams.ulOffset = ulOffset;

	JR3PCI_READ_WORD_RESPONSE_PARAMS ReadWordResponseParams;

	DWORD dwBytesReturned = 0;
	BOOL success = DeviceIoControl(
		hJr3PciDevice,								// handle to device
		IOCTL_JR3PCI_READ_WORD,						// operation
		&ReadWordRequestParams,						// input data buffer
		sizeof(JR3PCI_READ_WORD_REQUEST_PARAMS),	// size of input data buffer
		&ReadWordResponseParams,					// output data buffer
		sizeof(JR3PCI_READ_WORD_RESPONSE_PARAMS),	// size of output data buffer
		&dwBytesReturned,							// byte count
		nullptr);									// overlapped information

	_ASSERTE(success && (dwBytesReturned == sizeof(JR3PCI_READ_WORD_RESPONSE_PARAMS)));
	_ASSERTE(ReadWordResponseParams.iStatus == JR3PCI_STATUS_OK);

	return ReadWordResponseParams.usData;
}


void WriteWord(HANDLE hJr3PciDevice, UCHAR ucChannel, ULONG ulOffset, USHORT usData)
{
	JR3PCI_WRITE_WORD_REQUEST_PARAMS WriteWordRequestParams;
	WriteWordRequestParams.ucChannel = ucChannel;
	WriteWordRequestParams.ulOffset = ulOffset;
	WriteWordRequestParams.usData = usData;

	JR3PCI_WRITE_WORD_RESPONSE_PARAMS WriteWordResponseParams;

	DWORD dwBytesReturned = 0;
	BOOL bSuccess = DeviceIoControl(
		hJr3PciDevice,								// handle to device
		IOCTL_JR3PCI_WRITE_WORD,					// operation
		&WriteWordRequestParams,					// input data buffer
		sizeof(JR3PCI_WRITE_WORD_REQUEST_PARAMS),	// size of input data buffer
		&WriteWordResponseParams,					// output data buffer
		sizeof(JR3PCI_WRITE_WORD_RESPONSE_PARAMS),	// size of output data buffer
		&dwBytesReturned,							// byte count
		nullptr);									// overlapped information

	_ASSERTE(bSuccess && (dwBytesReturned == sizeof(JR3PCI_WRITE_WORD_RESPONSE_PARAMS)));
	_ASSERTE(WriteWordResponseParams.iStatus == JR3PCI_STATUS_OK);
}


} // extern "C"


namespace franka_proxy
{


ft_sensor_jr3::ft_sensor_jr3()
	: device_name_(R"(\\.\JR3PCI1)")
{
	init_jr3();
	set_offsets_to_zero();
	update();
}


void ft_sensor_jr3::set_offsets_to_zero()
{
	update();
	reset_offsets();

	std::array<std::vector<int>, 6> raw_values;
	for (int i = 0; i < 10000; ++i)
	{
		update();

		for (int i = 0; i < 6; ++i)
			raw_values[i].emplace_back(current_raw_values_f0()[i]);
	}

	std::array<int, 6> offsets{};
	for (int i = 0; i < 6; ++i)
	{
		offsets[i] = std::accumulate(
			raw_values[i].begin(),
			raw_values[i].end(), 0) / static_cast<int>(raw_values[i].size());

		// add current offset
		const ULONG offset_f0 = 0x88 + i;
		offsets[i] += ReadWord(jr3_pci_device_, channel, offset_f0);

		WriteWord(jr3_pci_device_, channel, offset_f0, offsets[i]);
	}
}


void ft_sensor_jr3::set_toolframe_transformation()
{
	throw std::exception("not implemented");
}


void ft_sensor_jr3::set_full_scales()
{
	throw std::exception("not implemented");
}


void ft_sensor_jr3::update()
{
	for (int i = 0; i < 6; i++)
	{
		const ULONG offset_f0 = 0x90 + i;
		const ULONG offset_f2 = 0xa0 + i;
		const ULONG offset_f3 = 0xa8 + i;
		const ULONG offset_f4 = 0xb0 + i;

		const SHORT data_f0 = ReadWord(jr3_pci_device_, channel, offset_f0);
		const SHORT data_f2 = ReadWord(jr3_pci_device_, channel, offset_f2);
		const SHORT data_f3 = ReadWord(jr3_pci_device_, channel, offset_f3);
		const SHORT data_f4 = ReadWord(jr3_pci_device_, channel, offset_f4);

		raw_values_f0_[i] = data_f0;
		raw_values_f2_[i] = data_f2;
		raw_values_f3_[i] = data_f3;
		raw_values_f4_[i] = data_f4;

		values_[i] = raw_values_f0_[i] * adapt[i];
	}
}


std::array<double, 6> ft_sensor_jr3::current_values() const
{
	return values_;
}


std::array<double, 6> ft_sensor_jr3::current_values_f2() const
{
	std::array<double, 6> values;

	for (int i = 0; i < 6; ++i)
		values[i] = raw_values_f2_[i] * adapt[i];

	return values;
}


std::array<int, 6> ft_sensor_jr3::current_raw_values_f0() const
{
	return raw_values_f0_;
}


std::array<int, 6> ft_sensor_jr3::current_raw_values_f2() const
{
	return raw_values_f2_;
}


std::array<int, 6> ft_sensor_jr3::current_raw_values_f3() const
{
	return raw_values_f3_;
}


std::array<int, 6> ft_sensor_jr3::current_raw_values_f4() const
{
	return raw_values_f4_;
}


void ft_sensor_jr3::init_jr3()
{
	jr3_pci_device_ = CreateFile(
		device_name_.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		nullptr,
		OPEN_EXISTING,
		0,
		nullptr);

	channel = 0;


	for (int i = 0; i < 6; ++i)
	{
		const ULONG offset_full_scale = 0x80 + i;
		const SHORT data_full_scale = ReadWord(jr3_pci_device_, channel, offset_full_scale);
		ranges[i] = static_cast<double>(data_full_scale);
	}


	for (int i = 0; i < 6; i++)
		adapt[i] = ranges[i] / full_scale;
}


void ft_sensor_jr3::reset_offsets()
{
	WriteWord(jr3_pci_device_, channel, offset_command0, 0x0800);
}


} // namespace franka_proxy
