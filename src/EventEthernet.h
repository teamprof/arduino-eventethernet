/* Copyright 2022 teamprof.net@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef __EventEthernet_h_
#define __EventEthernet_h_

#include <Arduino.h>
#include <mbed.h>

#include "Ethernet.h"
#define PIN_W5100S_CSn 17  // W5100S-EVB-Pico: nCS pin = GPIO17
#define Pin_W5100S_Intn 21 // W5100S-EVB-Pico: INTn pin = GPIO21

// using namespace mbed;
using namespace rtos;
using namespace events;

class IR
{
public:
	// IR (Interrupt Register) [RW] [0x0015] [0x00]
	// bit7: CONFLICT
	// bit6: UNREACH
	// BIT5: PPPTERM
	// bit4:
	// bit3-0: Sn_INT     Each n-th Bit describes SOCKET n-th Interrupt.
	static const uint8_t CONFLICT = 0x80;
	static const uint8_t UNREACH = 0x40;
	static const uint8_t PPPTERM = 0x20;
	static const uint8_t S3_INT = 0x08;
	static const uint8_t S2_INT = 0x04;
	static const uint8_t S1_INT = 0x02;
	static const uint8_t S0_INT = 0x01;
};
class IR2
{
public:
	// IR2 (Interrupt Register 2) [RW] [0x0020] [0x00]
	// bit0: WOL            1 : Received UDP based WOL Magic Packet.
	static const uint8_t WOL = 0x01;
};
class SLIR
{
public:
	// SLIR (SOCKET-less Interrupt Register) [RW] [0x005F] [0x00]
	// bit2: TIMEOUT
	// bit1: ARP
	// BIT0: PING
	static const uint8_t TIMEOUT = 0x04;
	static const uint8_t ARP = 0x02;
	static const uint8_t PING = 0x01;
};
#define Sn_INT (IR::S3_INT | IR::S2_INT | IR::S1_INT | IR::S0_INT)

class SocketEventApi
{
public:
	SocketEventApi();
	~SocketEventApi();

	typedef void (*SocketEventCallback)(uint8_t sn_ir);
	virtual bool subscribeSocketEvent(uint8_t sn_ir, SocketEventCallback onSocketEvent, uint8_t sockindex);
	virtual void unsubscribeSocketEvent(uint8_t sn_ir, uint8_t sockindex);

protected:
	// uint8_t sockIndex; // MAX_SOCK_NUM means client not in use
	uint8_t sn_ir;
	SocketEventCallback eventCallback;
};

class IMR
{
public:
	static const uint8_t CNFT = 0x80;
	static const uint8_t UNREACH = 0x40;
	static const uint8_t PPPTERM = 0x20;
	static const uint8_t S3_INT = 0x08;
	static const uint8_t S2_INT = 0x04;
	static const uint8_t S1_INT = 0x02;
	static const uint8_t S0_INT = 0x01;
};

class IMR2
{
public:
	static const uint8_t WOL = 0x01;
};

class SLIMR
{
public:
	static const uint8_t TIMEOUT = 0x04;
	static const uint8_t ARP = 0x02;
	static const uint8_t PING = 0x01;
};

class SnIMR
{
public:
	static const uint8_t SEND_OK = 0x10;
	static const uint8_t TIMEOUT = 0x08;
	static const uint8_t RECV = 0x04;
	static const uint8_t DISCON = 0x02;
	static const uint8_t CON = 0x01;
};

///////////////////////////////////////////////////////////////////////////////
// EventEthernetUDP
///////////////////////////////////////////////////////////////////////////////
#define SnIR_MASK (SnIR::SEND_OK | SnIR::TIMEOUT | SnIR::RECV | SnIR::DISCON | SnIR::CON)
class EventEthernetUDP : public EthernetUDP, SocketEventApi
{
public:
	virtual void begin(uint16_t port, uint8_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr);
	virtual uint8_t beginMulticast(IPAddress ip, uint16_t port, uint8_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr); // initialize, start listening on specified port. Returns 1 if successful, 0 if there are no sockets available to use
	virtual void stop();
};

///////////////////////////////////////////////////////////////////////////////
// EventEthernetClient
///////////////////////////////////////////////////////////////////////////////
class EventEthernetClient : public EthernetClient, SocketEventApi
{
public:
	EventEthernetClient() : EthernetClient() {}
	EventEthernetClient(uint8_t s) : EthernetClient(s) {}
	virtual int connect(IPAddress ip, uint16_t port, uint8_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr);
	virtual int connect(const char *host, uint16_t port, uint8_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr);
	virtual void stop();
};

///////////////////////////////////////////////////////////////////////////////
// EventEthernetServer
///////////////////////////////////////////////////////////////////////////////
class EventEthernetServer : public EthernetServer, SocketEventApi
{
public:
	EventEthernetServer(uint16_t port);
	~EventEthernetServer();
	EventEthernetClient available();
	EventEthernetClient accept();
	virtual void begin(uint8_t sn_ir = 0, SocketEventCallback onSocketEvent = nullptr);

	virtual bool subscribeSocketEvent(uint8_t sn_ir, SocketEventCallback onSocketEvent);
	virtual void unsubscribeSocketEvent(uint8_t sn_ir);

private:
	uint16_t _serverPort;
};

///////////////////////////////////////////////////////////////////////////////
// EventEthernetClass
///////////////////////////////////////////////////////////////////////////////
class EventEthernetClass : public EthernetClass
{
public:
	typedef void (*EthernetEventCallback)(uint8_t ir, uint8_t ir2, uint8_t slir);

	EventEthernetClass();
	~EventEthernetClass();
	static int begin(uint8_t *mac, uint8_t ir = 0, uint8_t ir2 = 0, uint8_t slir = 0, EthernetEventCallback onEthernetEvent = nullptr);
	static int begin(uint8_t *mac, unsigned long timeout = 60000, uint8_t ir = 0, uint8_t ir2 = 0, uint8_t slir = 0, EthernetEventCallback onEthernetEvent = nullptr);
	static int begin(uint8_t *mac, unsigned long timeout = 60000, unsigned long responseTimeout = 4000, uint8_t ir = 0, uint8_t ir2 = 0, uint8_t slir = 0, EthernetEventCallback onEthernetEvent = nullptr);
	static void begin(uint8_t *mac, IPAddress ip, uint8_t ir = 0, uint8_t ir2 = 0, uint8_t slir = 0, EthernetEventCallback onEthernetEvent = nullptr);
	static void begin(uint8_t *mac, IPAddress ip, IPAddress dns, uint8_t ir = 0, uint8_t ir2 = 0, uint8_t slir = 0, EthernetEventCallback onEthernetEvent = nullptr);
	static void begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, uint8_t ir = 0, uint8_t ir2 = 0, uint8_t slir = 0, EthernetEventCallback onEthernetEvent = nullptr);
	static void begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet, uint8_t ir = 0, uint8_t ir2 = 0, uint8_t slir = 0, EthernetEventCallback onEthernetEvent = nullptr);
	// static void init(uint8_t sspin = PIN_W5100S_CSn, uint8_t pinIntn = Pin_W5100S_Intn);
	void init(uint8_t sspin = PIN_W5100S_CSn, int8_t pinIntn = -1);

	void subscribeEthernetEvent(uint8_t ir, uint8_t ir2, uint8_t slir, EthernetEventCallback onEthernetEvent);
	void unsubscribeEthernetEvent(uint8_t ir, uint8_t ir2, uint8_t slir);

	bool registerSocketCallback(uint8_t sockindex, SocketEventApi::SocketEventCallback onSocketEvent);
	void unregisterSocketCallback(uint8_t sockindex);

	friend class EthernetClient;
	friend class EthernetServer;
	friend class EthernetUDP;

private:
	uint8_t ir;
	uint8_t ir2;
	uint8_t slir;
	EthernetEventCallback ethernetEventCallback;
	SocketEventApi::SocketEventCallback socketEventCallback[MAX_SOCK_NUM];
	// void (*ethernetEventCallback)(uint8_t, uint8_t, uint8_t);
	// void (*socketEventCallback[MAX_SOCK_NUM])(uint8_t);

	int8_t pinIntn;
	// static EventEthernetClass *instance;
	static void isr(EventEthernetClass *);
	void queueIsr(void);

	Thread thread;
	EventQueue queue;
};

extern EventEthernetClass Ethernet;

#endif
