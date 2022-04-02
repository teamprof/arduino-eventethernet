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

#include <Arduino.h>
#include "EventEthernet.h"
// #include "Ethernet.h"
#include "utility/w5100.h"

// Disable Logging Macro (Release Mode)
#define DEBUGLOG_DISABLE_LOG
// You can also set default log level by defining macro (default: INFO)
// #define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE
#include <DebugLog.h> // https://github.com/hideakitai/DebugLog

///////////////////////////////////////////////////////////////////////////////
// SocketEventApi
///////////////////////////////////////////////////////////////////////////////
static const uint8_t socketIndexToImrBitTable[] = {
	0x01, 0x02, 0x04, 0x08 // for W5100S only
};

SocketEventApi::SocketEventApi()
{
	// sockIndex = MAX_SOCK_NUM;
	sn_ir = 0;
	eventCallback = nullptr;
}
SocketEventApi::~SocketEventApi()
{
	// unsubscribeSocketEvent(sn_ir, sockIndex);
	// sockIndex = MAX_SOCK_NUM;
	sn_ir = 0;
	eventCallback = nullptr;
}

bool SocketEventApi::subscribeSocketEvent(uint8_t sn_ir, SocketEventCallback onSocketEvent, uint8_t sockindex)
{
	LOG_DEBUG("sn_ir = ", DebugLogBase::HEX, sn_ir, "(hex), onSocketEvent = ", DebugLogBase::HEX, (uint32_t)(onSocketEvent), "(hex), sockindex = ", DebugLogBase::DEC, sockindex);

	bool ret = false;
	sn_ir = sn_ir & SnIR_MASK;
	if (sn_ir && (onSocketEvent != nullptr) && sockindex < MAX_SOCK_NUM)
	{
		// this->sockIndex = sockindex;
		this->eventCallback = onSocketEvent;
		ret = Ethernet.registerSocketCallback(sockindex, onSocketEvent);

		uint8_t sn_imr = W5100.readSnIMR_W5100S(sockindex);
		if (sn_ir & SnIR::SEND_OK)
		{
			sn_imr |= SnIMR::SEND_OK;
		}
		if (sn_ir & SnIR::TIMEOUT)
		{
			sn_imr |= SnIMR::TIMEOUT;
		}
		if (sn_ir & SnIR::RECV)
		{
			sn_imr |= SnIMR::RECV;
		}
		if (sn_ir & SnIR::DISCON)
		{
			sn_imr |= SnIMR::DISCON;
		}
		if (sn_ir & SnIR::CON)
		{
			sn_imr |= SnIMR::CON;
		}
		W5100.writeSnIMR_W5100S(sockindex, sn_imr);

		uint8_t imr = W5100.readIMR();
		imr |= socketIndexToImrBitTable[sockindex];
		W5100.writeIMR(imr);

		this->sn_ir |= sn_ir;
		LOG_DEBUG("this->sn_ir = ", DebugLogBase::HEX, this->sn_ir, "(hex), this->eventCallback = ", DebugLogBase::HEX, (uint32_t)(this->eventCallback), "(hex)");
	}
	return ret;
}

void SocketEventApi::unsubscribeSocketEvent(uint8_t sn_ir, uint8_t sockindex)
{
	LOG_DEBUG("sn_ir = ", DebugLogBase::HEX, sn_ir, "(hex), sockindex = ", DebugLogBase::DEC, sockindex);

	sn_ir = sn_ir & SnIR_MASK;
	if (sn_ir && sockindex < MAX_SOCK_NUM)
	{
		uint8_t sn_imr = W5100.readSnIMR_W5100S(sockindex);
		if (sn_ir & SnIR::SEND_OK)
		{
			sn_imr &= ~SnIMR::SEND_OK;
		}
		if (sn_ir & SnIR::TIMEOUT)
		{
			sn_imr &= ~SnIMR::TIMEOUT;
		}
		if (sn_ir & SnIR::RECV)
		{
			sn_imr &= ~SnIMR::RECV;
		}
		if (sn_ir & SnIR::DISCON)
		{
			sn_imr &= ~SnIMR::DISCON;
		}
		if (sn_ir & SnIR::CON)
		{
			sn_imr &= ~SnIMR::CON;
		}
		W5100.writeSnIMR_W5100S(sockindex, sn_imr);

		if (!sn_imr)
		{
			uint8_t imr = W5100.readIMR();
			imr &= ~(socketIndexToImrBitTable[sockindex]);
			W5100.writeIMR(imr);
		}

		this->sn_ir &= ~sn_ir;
		// this->sockIndex = MAX_SOCK_NUM;
		if (!this->sn_ir)
		{
			Ethernet.unregisterSocketCallback(sockindex);
			this->eventCallback = nullptr;
		}
		LOG_DEBUG("this->sn_ir = ", DebugLogBase::HEX, this->sn_ir, "(hex), this->eventCallback = ", DebugLogBase::HEX, (uint32_t)(this->eventCallback), "(hex)");
	}
}

///////////////////////////////////////////////////////////////////////////////
// EventEthernetServer
///////////////////////////////////////////////////////////////////////////////
EventEthernetServer::EventEthernetServer(uint16_t port) : EthernetServer(port)
{
	memset(server_port, 0, sizeof(server_port));
	this->_serverPort = port;
}
EventEthernetServer::~EventEthernetServer()
{
	unsubscribeSocketEvent(this->sn_ir);
	this->_serverPort = 0;
}

EventEthernetClient EventEthernetServer::available()
{
	bool listening = false;
	uint8_t sockindex = MAX_SOCK_NUM;

	// W5100 chip does not support more than 4 sockets
	uint8_t chip = W5100.getChip();
	int maxindex = (chip == 51 || chip == 50) ? 4 : MAX_SOCK_NUM;
	for (uint8_t i = 0; i < maxindex; i++)
	{
		if (server_port[i] == _port)
		{
			uint8_t stat = Ethernet.socketStatus(i);
			if (stat == SnSR::ESTABLISHED || stat == SnSR::CLOSE_WAIT)
			{
				if (Ethernet.socketRecvAvailable(i) > 0)
				{
					sockindex = i;
				}
				else
				{
					// remote host closed connection, our end still open
					if (stat == SnSR::CLOSE_WAIT)
					{
						Ethernet.socketDisconnect(i);
						// status becomes LAST_ACK for short time
					}
				}
			}
			else if (stat == SnSR::LISTEN)
			{
				listening = true;
			}
			else if (stat == SnSR::CLOSED)
			{
				server_port[i] = 0;
			}
		}
	}
	if (!listening)
	{
		LOG_DEBUG("socket is NOT listening! sockindex = ", sockindex);
		LOG_DEBUG("_port = ", _port);
		LOG_DEBUG("server_port: ", LOG_AS_ARR(server_port, MAX_SOCK_NUM));
		begin(this->sn_ir, this->eventCallback);
	}
	LOG_DEBUG("return EventEthernetClient(", sockindex, ")");
	return EventEthernetClient(sockindex);
}

EventEthernetClient EventEthernetServer::accept()
{
	bool listening = false;
	uint8_t sockindex = MAX_SOCK_NUM;

	// W5100 chip does not support more than 4 sockets
	uint8_t chip = W5100.getChip();
	int maxindex = (chip == 51 || chip == 50) ? 4 : MAX_SOCK_NUM;
	for (uint8_t i = 0; i < maxindex; i++)
	{
		if (server_port[i] == _serverPort)
		{
			uint8_t stat = Ethernet.socketStatus(i);
			if (sockindex == MAX_SOCK_NUM &&
				(stat == SnSR::ESTABLISHED || stat == SnSR::CLOSE_WAIT))
			{
				// Return the connected client even if no data received.
				// Some protocols like FTP expect the server to send the
				// first data.
				sockindex = i;
				server_port[i] = 0; // only return the client once
			}
			else if (stat == SnSR::LISTEN)
			{
				listening = true;
			}
			else if (stat == SnSR::CLOSED)
			{
				server_port[i] = 0;
			}
		}
	}
	if (!listening)
	{
		LOG_DEBUG("socket is NOT listening! sockindex = ", sockindex);
		LOG_DEBUG("_port = ", _port);
		LOG_DEBUG("server_port: ", LOG_AS_ARR(server_port, MAX_SOCK_NUM));
		begin(this->sn_ir, this->eventCallback);
	}
	if (sockindex >= MAX_SOCK_NUM)
	{
		LOG_DEBUG("_port = ", _port);
		LOG_DEBUG("server_port: ", LOG_AS_ARR(server_port, MAX_SOCK_NUM));
	}
	LOG_DEBUG("return EventEthernetClient(", sockindex, ")");
	return EventEthernetClient(sockindex);
}
void EventEthernetServer::begin(uint8_t sn_ir, SocketEventCallback onSocketEvent)
{
	EthernetServer::begin();
	subscribeSocketEvent(sn_ir, onSocketEvent);
}

bool EventEthernetServer::subscribeSocketEvent(uint8_t sn_ir, SocketEventCallback onSocketEvent)
{
	bool ret = true;
	if (sn_ir && (onSocketEvent != nullptr))
	{
		for (uint8_t index = 0; index < MAX_SOCK_NUM; index++)
		{
			if (server_port[index] == _serverPort)
			{
				ret = SocketEventApi::subscribeSocketEvent(sn_ir, onSocketEvent, index);
			}
		}
	}
	return ret;
}
void EventEthernetServer::unsubscribeSocketEvent(uint8_t sn_ir)
{

	// W5100 chip does not support more than 4 sockets
	uint8_t chip = W5100.getChip();
	int maxindex = (chip == 51 || chip == 50) ? 4 : MAX_SOCK_NUM;
	for (uint8_t index = 0; index < maxindex; index++)
	{
		if (server_port[index] == _serverPort)
		{
			SocketEventApi::unsubscribeSocketEvent(sn_ir, index);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// EventEthernetClient
///////////////////////////////////////////////////////////////////////////////
int EventEthernetClient::connect(IPAddress ip, uint16_t port, uint8_t sn_ir, SocketEventCallback onSocketEvent)
{
	int ret = EthernetClient::connect(ip, port);
	SocketEventApi::subscribeSocketEvent(sn_ir, eventCallback, getSocketNumber());
	return ret;
}
int EventEthernetClient::connect(const char *host, uint16_t port, uint8_t sn_ir, SocketEventCallback onSocketEvent)
{
	int ret = EthernetClient::connect(host, port);
	SocketEventApi::subscribeSocketEvent(sn_ir, onSocketEvent, getSocketNumber());
	return ret;
}
void EventEthernetClient::stop()
{
	SocketEventApi::unsubscribeSocketEvent(this->sn_ir, getSocketNumber());
	EthernetClient::stop();
}

///////////////////////////////////////////////////////////////////////////////
// EventEthernetUDP
///////////////////////////////////////////////////////////////////////////////
void EventEthernetUDP::begin(uint16_t port, uint8_t sn_ir, SocketEventCallback onSocketEvent)
{
	EthernetUDP::begin(port);
	SocketEventApi::subscribeSocketEvent(sn_ir, onSocketEvent, this->sockindex);
}
uint8_t EventEthernetUDP::beginMulticast(IPAddress ip, uint16_t port, uint8_t sn_ir, SocketEventCallback onSocketEvent)
{
	EthernetUDP::beginMulticast(ip, port);
	SocketEventApi::subscribeSocketEvent(sn_ir, onSocketEvent, this->sockindex);
}
void EventEthernetUDP::stop(void)
{
	SocketEventApi::unsubscribeSocketEvent(this->sn_ir, this->sockindex);
	EthernetUDP::stop();
}

///////////////////////////////////////////////////////////////////////////////
// EventEthernetClass
///////////////////////////////////////////////////////////////////////////////
extern EventEthernetClass *instance __attribute__((weak));

#define EventEthernetThreadQueueSize (128 * EVENTS_EVENT_SIZE)
EventEthernetClass::EventEthernetClass() : queue(EventEthernetThreadQueueSize)
{
	instance = this;
	pinIntn = -1;
	ir = 0;
	ir2 = 0;
	slir = 0;

	// attachInterrupt(pinIntn, isr, FALLING, this);
	thread.start(callback(&queue, &EventQueue::dispatch_forever));
}
EventEthernetClass::~EventEthernetClass()
{
	if (pinIntn >= 0)
	{
		detachInterrupt(pinIntn);
	}
	instance = nullptr;
	unsubscribeEthernetEvent(ir, ir2, slir);
}

int EventEthernetClass::begin(uint8_t *mac, uint8_t ir, uint8_t ir2, uint8_t slir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac);
	Ethernet.subscribeEthernetEvent(ir, ir2, slir, onEthernetEvent);
}
int EventEthernetClass::begin(uint8_t *mac, unsigned long timeout, uint8_t ir, uint8_t ir2, uint8_t slir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, timeout);
	Ethernet.subscribeEthernetEvent(ir, ir2, slir, onEthernetEvent);
}
int EventEthernetClass::begin(uint8_t *mac, unsigned long timeout, unsigned long responseTimeout, uint8_t ir, uint8_t ir2, uint8_t slir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, timeout, responseTimeout);
	Ethernet.subscribeEthernetEvent(ir, ir2, slir, onEthernetEvent);
}
void EventEthernetClass::begin(uint8_t *mac, IPAddress ip, uint8_t ir, uint8_t ir2, uint8_t slir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, ip);
	Ethernet.subscribeEthernetEvent(ir, ir2, slir, onEthernetEvent);
}
void EventEthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, uint8_t ir, uint8_t ir2, uint8_t slir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, ip, dns);
	Ethernet.subscribeEthernetEvent(ir, ir2, slir, onEthernetEvent);
}
void EventEthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, uint8_t ir, uint8_t ir2, uint8_t slir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, ip, dns, gateway);
	Ethernet.subscribeEthernetEvent(ir, ir2, slir, onEthernetEvent);
}
void EventEthernetClass::begin(uint8_t *mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet, uint8_t ir, uint8_t ir2, uint8_t slir, EthernetEventCallback onEthernetEvent)
{
	EthernetClass::begin(mac, ip, dns, gateway, subnet);
	Ethernet.subscribeEthernetEvent(ir, ir2, slir, onEthernetEvent);
}

void EventEthernetClass::init(uint8_t sspin, int8_t pinIntn)
{
	EthernetClass::init(sspin);
	this->pinIntn = pinIntn;
	if (pinIntn >= 0)
	{
		// Wiznet W5100S INTn is a level interrupt (active LOW),
		// attach as "FALLING" instead of "LOW" is a workaround
		// to prevent infinite interrupts durng the time between isr and queueIsr
		attachInterrupt(pinIntn, isr, FALLING, this);
		// attachInterrupt(pinIntn, isr, FALLING, instance);
	}
}

#define dim(x) (sizeof(x) / sizeof(x[0]))
void EventEthernetClass::subscribeEthernetEvent(uint8_t ir, uint8_t ir2, uint8_t slir, EthernetEventCallback onEthernetEvent)
{
	// mask Sn_INT which is enable/disable in registerSocketCallback()/unregisterSocketCallback()
	ir &= ~Sn_INT;

	this->ir |= ir;
	this->ir2 |= ir2;
	this->slir |= slir;

	if (ir | ir2 | slir)
	{
		ethernetEventCallback = onEthernetEvent;
	}

	uint8_t imr = W5100.readIMR();
	if (ir & IR::CONFLICT)
	{
		imr |= IMR::CNFT;
	}
	if (ir & IR::UNREACH)
	{
		imr |= IMR::UNREACH;
	}
	if (ir & IR::PPPTERM)
	{
		imr |= IMR::PPPTERM;
	}
	W5100.writeIMR(imr);

	uint8_t imr2 = W5100.readIMR2_W5100S();
	if (ir2 & IR2::WOL)
	{
		imr2 |= IMR2::WOL;
	}
	W5100.writeIMR2_W5100S(imr2);

	uint8_t slimr = W5100.readSLIMR_W5100S();
	if (slir & SLIR::TIMEOUT)
	{
		slimr |= SLIMR::TIMEOUT;
	}
	if (slir & SLIR::ARP)
	{
		slimr |= SLIMR::ARP;
	}
	if (slir & SLIR::PING)
	{
		slimr |= SLIMR::PING;
	}
	W5100.writeSLIMR_W5100S(slimr);

	LOG_DEBUG("this->ir = ", DebugLogBase::HEX, this->ir, "(hex), this->ir2 = ", DebugLogBase::HEX, this->ir2, "(hex) this->slir = ", DebugLogBase::HEX, this->slir, "(hex)");
}

void EventEthernetClass::unsubscribeEthernetEvent(uint8_t ir, uint8_t ir2, uint8_t slir)
{
	uint8_t imr = W5100.readIMR();
	if (ir & IR::CONFLICT)
	{
		imr &= ~IMR::CNFT;
	}
	if (ir & IR::UNREACH)
	{
		imr &= ~IMR::UNREACH;
	}
	if (ir & IR::PPPTERM)
	{
		imr &= ~IMR::PPPTERM;
	}
	W5100.writeIMR(imr);

	uint8_t imr2 = W5100.readIMR2_W5100S();
	if (ir2 & IR2::WOL)
	{
		imr2 &= ~IMR2::WOL;
	}
	W5100.writeIMR2_W5100S(imr2);

	uint8_t slimr = W5100.readSLIMR_W5100S();
	if (slir & SLIR::TIMEOUT)
	{
		slimr &= ~SLIMR::TIMEOUT;
	}
	if (slir & SLIR::ARP)
	{
		slimr &= ~SLIMR::ARP;
	}
	if (slir & SLIR::PING)
	{
		slimr &= ~SLIMR::PING;
	}
	W5100.writeSLIMR_W5100S(slimr);

	// ignore Sn_INT which is handled by registerSocketCallback()/unregisterSocketCallback()
	ir &= ~Sn_INT;
	imr &= ~Sn_INT;
	if (!imr && !imr2 && !slimr)
	{
		ethernetEventCallback = nullptr;
	}

	this->ir &= ~ir;
	this->ir2 &= ~ir2;
	this->slir &= ~slir;

	LOG_DEBUG("this->ir = ", DebugLogBase::HEX, this->ir, "(hex), this->ir2 = ", DebugLogBase::HEX, this->ir2, "(hex) this->slir = ", DebugLogBase::HEX, this->slir, "(hex)");
}

bool EventEthernetClass::registerSocketCallback(uint8_t sockindex, SocketEventApi::SocketEventCallback onSocketEvent)
{
	if (sockindex < MAX_SOCK_NUM)
	{
		socketEventCallback[sockindex] = onSocketEvent;
		return true;
	}
	return false;
}
void EventEthernetClass::unregisterSocketCallback(uint8_t sockindex)
{
	if (sockindex < MAX_SOCK_NUM)
	{
		socketEventCallback[sockindex] = nullptr;
	}
}

void EventEthernetClass::isr(EventEthernetClass *instance)
{
	if (instance != nullptr)
	{
		auto ev = instance->queue.event(instance, &EventEthernetClass::queueIsr);
		ev.post();
	}
}
// EventEthernetClass *EventEthernetClass::instance = nullptr;
// void EventEthernetClass::isr(EventEthernetClass *inst)
// {
// 	if (instance != nullptr)
// 	{
// 		auto ev = instance->queue.event(instance, &EventEthernetClass::queueIsr);
// 		ev.post();
// 	}
// }

void EventEthernetClass::queueIsr(void)
{
	while (digitalRead(pinIntn) == LOW)
	{
		// read registers and clear interrupt registers
		uint8_t ir = W5100.readIR();
		uint8_t ir2 = W5100.readIR2_W5100S();
		uint8_t slir = W5100.readSLIR_W5100S();

		uint8_t ir_eth = ir & ~Sn_INT;
		if ((ir_eth || ir2 || slir) && (ethernetEventCallback != nullptr))
		{
			auto ethEvent = queue.event(ethernetEventCallback);
			ethEvent.post(ir, ir2, slir);
		}

		uint8_t chip = W5100.getChip();
		uint8_t ir_sock = ir & Sn_INT;
		if (ir_sock && chip)
		{
			// W5100 chip never supports more than 4 sockets
			int maxindex = (chip == 51 || chip == 50) ? 4 : MAX_SOCK_NUM;
			for (int i = 0; i < maxindex; i++)
			{
				if (ir_sock & 0x01)
				{
					uint8_t sn_ir = W5100.readSnIR(i);
					uint8_t sn_sr = W5100.readSnSR(i);
					W5100.writeSnIR(i, sn_ir);

					auto eventCallback = socketEventCallback[i];
					if (eventCallback != nullptr)
					{
						auto sockEvent = queue.event(eventCallback);
						sockEvent.post(sn_ir);
					}
				}
				ir_sock >>= 1;
			}
		}

		// clear interrupt registers
		W5100.writeIR(ir);
		W5100.writeIR2_W5100S(ir2);
		W5100.writeSLIR_W5100S(slir);

		rtos::ThisThread::sleep_for(1);

		// if (digitalRead(pinIntn) == LOW)
		// {
		// 	// developer has to fix the "INTn LOW even clearing interrupt registers" error
		// 	LOG_ERROR("W5100S INTn pin keeps LOW even clearing interrupt registers!");
		// }
	}
}

EventEthernetClass Ethernet;
