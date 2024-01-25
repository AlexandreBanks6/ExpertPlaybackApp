#ifndef DECKLINKINPUTDEVICE_H
#define DECKLINKINPUTDEVICE_H

#include "DeckLinkDevice.h"

#define DEFAULT_DISPLAY_MODE bmdModeHD1080p5994

class BLACKMAGICCAPTURE_API DeckLinkInputDevice : public DeckLinkDevice, public IDeckLinkInputCallback
{
private:
	IDeckLinkInput* m_decklinkInput;

	BMDDisplayMode m_display_mode;

	void(*image_data_callback)(void * arg, void ** imagedata, int width, int height, int size, int port);
	void *arguments;

public:
	DeckLinkInputDevice(IDeckLink* dl, int port, BMDDisplayMode displayMode);
	~DeckLinkInputDevice();

	void Start();
	void Stop();
	void SetCallback(void * arg, void(*image_callback)(void * arg, void ** imagedata, int width, int height, int size, int port));

	// IUnknown interface
	virtual HRESULT	STDMETHODCALLTYPE	QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
	virtual ULONG	STDMETHODCALLTYPE	AddRef() { return 1; }
	virtual ULONG	STDMETHODCALLTYPE	Release() { return 1; }

	// IDeckLinkInputCallback interface
	virtual HRESULT VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame, IDeckLinkAudioInputPacket* audioPacket);
	virtual HRESULT	VideoInputFormatChanged(/* in */ BMDVideoInputFormatChangedEvents notificationEvents, /* in */ IDeckLinkDisplayMode *newMode, /* in */ BMDDetectedVideoInputFormatFlags detectedSignalFlags);
};

#endif
