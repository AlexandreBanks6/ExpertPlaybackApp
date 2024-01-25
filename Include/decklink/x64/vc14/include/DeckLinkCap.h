#ifndef DECKLINKCAP_H
#define DECKLINKCAP_H


#include "DeckLinkAPI_h.h"

#include "DeckLinkInputDevice.h"
#include "DeckLinkOutputDevice.h"

#ifdef BLACKMAGICCAPTURE_EXPORTS  
#define BLACKMAGICCAPTURE_API __declspec(dllexport)   
#else  
#define BLACKMAGICCAPTURE_API __declspec(dllimport)   
#endif 

class BLACKMAGICCAPTURE_API DeckLinkCap
{
public:
	DeckLinkCap();
	~DeckLinkCap();

	DeckLinkInputDevice* InitInputAtPort(int port, BMDDisplayMode displayMode = DEFAULT_DISPLAY_MODE);
	DeckLinkOutputDevice* InitOutputAtPort(int port);
	bool RemoveDeviceAtPort(int port);

	bool IsInitialized() { return m_initialized; }

private:
	IDeckLink*	m_deckLink[16];

	DeckLinkDevice*	m_deckLinkDeviceList[8] = { 0 };

	bool		m_initialized;
};

#endif
