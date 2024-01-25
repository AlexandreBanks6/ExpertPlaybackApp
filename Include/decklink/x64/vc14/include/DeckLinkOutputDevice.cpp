#include <comutil.h>
#include "DeckLinkOutputDevice.h"

using namespace std;

// 3090 robot (DVRK)
// Video mode parameters
const BMDDisplayMode      kDisplayMode = bmdModeNTSC;
const BMDVideoOutputFlags kOutputFlags = bmdVideoOutputVITC;
const BMDPixelFormat      kPixelFormat = bmdFormat8BitYUV;

// Values taken from DL DisplayMode object for bmdModeNTSC
const unsigned kFrameDuration = 1001;
const unsigned kTimeScale = 30000;
const unsigned kFrameWidth = 720;
const unsigned kFrameHeight = 486;
const unsigned kRowBytes = 720 * 2;

/*
//3080 robot (S)
// Video mode parameters
const BMDDisplayMode      kDisplayMode = bmdModeHD1080i5994;
const BMDVideoOutputFlags kOutputFlags = bmdVideoOutputFlagDefault;
const BMDPixelFormat      kPixelFormat = bmdFormat8BitBGRA;

// Values taken from DL DisplayMode object for bmdModeHD1080i5994
const unsigned kFrameDuration = 1001;
const unsigned kTimeScale = 30000;
const unsigned kFrameWidth = 1920;
const unsigned kFrameHeight = 1080;
const unsigned kRowBytes = 7680;
*/
DeckLinkOutputDevice::DeckLinkOutputDevice(IDeckLink* dl, int port)
{
	m_deckLinkOutput = NULL;

	m_frame_width = 1920;
	m_frame_height = 1080;

	rd_buffer_count = 0;
	wr_buffer_count = 0;

	videoFrameBlue = NULL;
	previousImage = NULL;

	gTotalFramesScheduled = 0;

	m_port = port;

	HRESULT result;
	result = dl->QueryInterface(IID_IDeckLinkOutput, (void**)&m_deckLinkOutput);
	if (result != S_OK) return;
}

DeckLinkOutputDevice::~DeckLinkOutputDevice()
{
	if (m_deckLinkOutput != NULL)
	{
		Stop();

		m_deckLinkOutput->Release();
		m_deckLinkOutput = NULL;
	}

	if (previousImage != NULL)
		delete[] previousImage;

	if (videoFrameBlue != NULL)
		videoFrameBlue->Release();

	if (buffers) {
		for (int i = 0; i < 2; i++)
		{
			if (buffers[i].buffer) buffers[i].buffer->Release();
		}
	}
}

void DeckLinkOutputDevice::Start()
{
	// Create a frame with defined format
	if (!videoFrameBlue)
	{
		videoFrameBlue = CreateFrame(m_deckLinkOutput);
	}

	for (int i = 0; i < 2; i++)
	{
		if (!buffers[i].buffer)
		{
			buffers[i].buffer = CreateFrame(m_deckLinkOutput);
		}
		buffers[i].dirty = 1;
	}

	m_deckLinkOutput->SetScheduledFrameCompletionCallback(this);
	m_deckLinkOutput->EnableVideoOutput(bmdModeHD1080i5994, bmdVideoOutputFlagDefault);
	m_deckLinkOutput->ScheduleVideoFrame(videoFrameBlue, gTotalFramesScheduled*kFrameDuration, kFrameDuration, kTimeScale);
	m_deckLinkOutput->StartScheduledPlayback(0, kTimeScale, 1.0);

	gTotalFramesScheduled++;
}

void DeckLinkOutputDevice::Stop()
{
	m_deckLinkOutput->StopScheduledPlayback(0, NULL, 0);
	m_deckLinkOutput->DisableVideoOutput();
	m_deckLinkOutput->SetScheduledFrameCompletionCallback(NULL);

	gTotalFramesScheduled = 0;
}

bool DeckLinkOutputDevice::UpdateBuffers(unsigned char * image_buffer, unsigned size)
{
	if (buffers[wr_buffer_count & 0x1].dirty)
	{
		void * frame;
		buffers[wr_buffer_count & 0x1].buffer->GetBytes(&frame);
		memcpy(frame, image_buffer, size);
		buffers[wr_buffer_count & 0x1].dirty = 0;
		wr_buffer_count++;
		return true;
	}
	else
	{
		printf("skip\n");
	}
	return false;
}

IDeckLinkMutableVideoFrame* DeckLinkOutputDevice::CreateFrame(IDeckLinkOutput* deckLinkOutput)
{
	IDeckLinkMutableVideoFrame* frame = NULL;
	HRESULT result = deckLinkOutput->CreateVideoFrame(kFrameWidth, kFrameHeight, kRowBytes, kPixelFormat, bmdFrameFlagFlipVertical, &frame);
	if (result == S_OK) {
		return frame;
	}
	else {
		return NULL;
	}
}

void DeckLinkOutputDevice::FillBlue(IDeckLinkMutableVideoFrame* theFrame)
{
	unsigned char * dat;
	unsigned row = theFrame->GetRowBytes();
	unsigned col = theFrame->GetHeight();

	theFrame->GetBytes((void**)&dat);

	memset(dat, 0, row*col);
	unsigned i = 0;
	for (i = 0; i < row*col; i = i + 4)
	{
		dat[i] = 255;
	}
}

HRESULT	STDMETHODCALLTYPE DeckLinkOutputDevice::ScheduledFrameCompleted(IDeckLinkVideoFrame* completedFrame, BMDOutputFrameCompletionResult result)
{
	// When a video frame completes,reschedule another frame
	void * pFrame;
	completedFrame->GetBytes(&pFrame);

	unsigned row = completedFrame->GetRowBytes();
	unsigned col = completedFrame->GetHeight();

	if (!previousImage) {
		previousImage = new unsigned char[row*col];
	}

	void * test_image;
	if (buffers[rd_buffer_count & 0x1].dirty == 0)
	{
		buffers[rd_buffer_count & 0x1].buffer->GetBytes(&test_image);
		memcpy(pFrame, test_image, row*col);
		buffers[rd_buffer_count & 0x1].dirty = 1;
		rd_buffer_count++;
		printf("Reading: %u\n", rd_buffer_count);
	}
	else
	{
		memcpy(pFrame, previousImage, row*col);
	}

	m_deckLinkOutput->ScheduleVideoFrame(completedFrame, gTotalFramesScheduled*kFrameDuration, kFrameDuration, kTimeScale);

	memcpy(previousImage, pFrame, row*col);
	gTotalFramesScheduled++;
	return S_OK;
}

HRESULT	STDMETHODCALLTYPE DeckLinkOutputDevice::ScheduledPlaybackHasStopped(void)
{
	return S_OK;
}
