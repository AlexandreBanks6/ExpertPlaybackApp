#include "DeckLinkCap.h"

DeckLinkCap::DeckLinkCap()
{
	CoInitializeEx(NULL, COINIT_MULTITHREADED);

	m_initialized = false;

	IDeckLinkIterator* pIterator = NULL;

	// Get list of decklink devices
	HRESULT result;
	result = CoCreateInstance(CLSID_CDeckLinkIterator, NULL, CLSCTX_ALL, IID_IDeckLinkIterator, (void**)&pIterator);
	if (FAILED(result))
	{
		return;
	}

	int i = 0;
	// Get reference to devices
	for (i = 0; i<16; i++)
	{
		if (pIterator->Next(&m_deckLink[i]) != S_OK)
			break;
	}
	pIterator->Release();

	// Check if number of devices > 0
	if (i == 0)
	{
		return ;
	}

	m_initialized = true;
}

DeckLinkCap::~DeckLinkCap()
{
	for (int i = 0; i < 8; i++)
	{
		if (m_deckLinkDeviceList[i])
		{
			m_deckLinkDeviceList[i]->Stop();
			delete m_deckLinkDeviceList[i];
			m_deckLinkDeviceList[i] = NULL;
		}
	}
}

DeckLinkInputDevice* DeckLinkCap::InitInputAtPort(int port, BMDDisplayMode displayMode)
{
	if (!m_initialized) return NULL;

	int port_index = port - 1;

	if (port_index < 0 || port_index > 8) return NULL;

	if (m_deckLinkDeviceList[port_index]) return NULL;

	DeckLinkInputDevice* input = new DeckLinkInputDevice(m_deckLink[port_index], port, displayMode);
	m_deckLinkDeviceList[port_index] = input;

	return input;
}

DeckLinkOutputDevice* DeckLinkCap::InitOutputAtPort(int port)
{
	if (!m_initialized) return NULL;

	int port_index = port - 1;

	if (port_index < 0 || port_index > 8) return NULL;

	if (m_deckLinkDeviceList[port_index]) return NULL;

	DeckLinkOutputDevice* output = new DeckLinkOutputDevice(m_deckLink[port_index], port);
	m_deckLinkDeviceList[port_index] = output;

	return output;
}

bool DeckLinkCap::RemoveDeviceAtPort(int port)
{
	if (!m_initialized) return false;

	int port_index = port - 1;

	if (port_index < 0 || port_index > 8) return false;

	if (!m_deckLinkDeviceList[port_index]) return false;

	auto device = m_deckLinkDeviceList[port_index];
	delete device;

	//delete m_deckLinkDeviceList[port_index];
	m_deckLinkDeviceList[port_index] = NULL;

	return true;
}