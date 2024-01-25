#include <comutil.h>
#include "DeckLinkInputDevice.h"

using namespace std;

DeckLinkInputDevice::DeckLinkInputDevice(IDeckLink* dl, int port, BMDDisplayMode displayMode)
{
	m_decklinkInput = NULL;
	image_data_callback = NULL;
	arguments = NULL;

	m_display_mode = displayMode;

	m_port = port;

	HRESULT result;
	result = dl->QueryInterface(IID_IDeckLinkInput, (void**)&m_decklinkInput);
	if (result != S_OK) return;
}

DeckLinkInputDevice::~DeckLinkInputDevice()
{
	if (m_decklinkInput != NULL)
	{
		Stop();

		m_decklinkInput->Release();
		m_decklinkInput = NULL;
	}
}

void DeckLinkInputDevice::Start()
{
	m_decklinkInput->SetCallback(this);
	m_decklinkInput->EnableVideoInput(m_display_mode, bmdFormat8BitYUV, bmdVideoInputFlagDefault);
	m_decklinkInput->StartStreams();
}

void DeckLinkInputDevice::Stop()
{
	m_decklinkInput->StopStreams();
	m_decklinkInput->DisableVideoInput();
	m_decklinkInput->SetCallback(NULL);
}

void DeckLinkInputDevice::SetCallback(void * arg, void(*image_callback)(void * arg, void ** imagedata, int width, int height, int size, int port))
{
	arguments = arg;
	image_data_callback = image_callback;
}

HRESULT DeckLinkInputDevice::VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame, IDeckLinkAudioInputPacket* audioPacket)
{
	void* buffer;
	videoFrame->GetBytes(&buffer);

	if (image_data_callback != NULL)
	{
		(*image_data_callback)(arguments, &buffer, videoFrame->GetWidth(), videoFrame->GetHeight(), videoFrame->GetRowBytes()*videoFrame->GetHeight(), m_port);
	}

	return S_OK;
}

HRESULT	DeckLinkInputDevice::VideoInputFormatChanged(/* in */ BMDVideoInputFormatChangedEvents notificationEvents, /* in */ IDeckLinkDisplayMode *newMode, /* in */ BMDDetectedVideoInputFormatFlags detectedSignalFlags)
{
	return S_OK;
}