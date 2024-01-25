#ifndef DECKLINKDEVICE_H
#define DECKLINKDEVICE_H


#include "DeckLinkAPI_h.h"

#ifdef BLACKMAGICCAPTURE_EXPORTS  
#define BLACKMAGICCAPTURE_API __declspec(dllexport)   
#else  
#define BLACKMAGICCAPTURE_API __declspec(dllimport)   
#endif 

class BLACKMAGICCAPTURE_API DeckLinkDevice
{
protected:
	int m_port;

	DeckLinkDevice() {}

public:
	virtual ~DeckLinkDevice() {}

	virtual void Start() {};
	virtual void Stop() {};

	// IUnknown interface
	virtual HRESULT	STDMETHODCALLTYPE	QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
	virtual ULONG	STDMETHODCALLTYPE	AddRef() { return 1; }
	virtual ULONG	STDMETHODCALLTYPE	Release() { return 1; }
};

#endif
