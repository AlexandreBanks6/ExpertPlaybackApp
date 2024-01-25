#ifndef DECKLINKOUTPUTDEVICE_H
#define DECKLINKOUTPUTDEVICE_H


#include "DeckLinkDevice.h"

typedef struct image_buffer
{
	IDeckLinkVideoFrame* buffer;
	unsigned dirty;
} image_buffer;

class BLACKMAGICCAPTURE_API DeckLinkOutputDevice : public DeckLinkDevice, public IDeckLinkVideoOutputCallback
{
private:
	IDeckLinkOutput* m_deckLinkOutput;

	int m_frame_width;
	int m_frame_height;

	image_buffer buffers[2];
	int wr_buffer_count;
	int rd_buffer_count;

	unsigned gTotalFramesScheduled;

	unsigned char * previousImage;
	IDeckLinkVideoFrame* videoFrameBlue;

	IDeckLinkMutableVideoFrame* CreateFrame(IDeckLinkOutput* deckLinkOutput);
	void FillBlue(IDeckLinkMutableVideoFrame* theFrame);

public:
	DeckLinkOutputDevice(IDeckLink* dl, int port);
	~DeckLinkOutputDevice();

	void Start();
	void Stop();

	bool UpdateBuffers(unsigned char * image_buffer, unsigned size);

	// IUnknown interface
	virtual HRESULT	STDMETHODCALLTYPE	QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
	virtual ULONG	STDMETHODCALLTYPE	AddRef() { return 1; }
	virtual ULONG	STDMETHODCALLTYPE	Release() { return 1; }

	// IDeckLinkOutputCallback interface
	virtual HRESULT	STDMETHODCALLTYPE ScheduledFrameCompleted(IDeckLinkVideoFrame* completedFrame, BMDOutputFrameCompletionResult result);
	virtual HRESULT	STDMETHODCALLTYPE ScheduledPlaybackHasStopped();
};

#endif
