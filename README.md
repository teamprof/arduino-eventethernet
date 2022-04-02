# arduino-eventethernet
## An event driven Ethernet library for Arduino, running on W5100S-EVB-Pico board
---

[![License: LGPL v3](https://img.shields.io/badge/License-LGPL_v3-blue.svg)](https://github.com/teamprof/arduino-eventethernet/blob/main/LICENSE)

<a href="https://www.buymeacoffee.com/teamprofnet" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" style="height: 38px !important;width: 168px !important;" ></a>

---
## Important Notes
This library is target for [W5100S-EVB-Pico] board at this moment.

This library is derived from a modified WIZnet-ArduinoEthernet library. In order for users to reuse their code (minimize user's code modification), the EventEthernet library overrides the modified WIZnet-ArduinoEthernet methods to implment the event driven features by handling W5100S interrupts.
 
### User's Code comparison: 
#### Using [EventEthernet](https://github.com/teamprof/arduino-eventethernet)
```
#include <EventEthernet.h>
void setup() {
  Ethernet.init(17, 21); // W5100S-EVB-Pico: nCS pin = GPIO17, Intn pin = GPIO21
  while (Ethernet.begin(mac, ir, ir2, slir, onEthernetEvent) == 0)
  {
    LOG_DEBUG("Failed to configure Ethernet using DHCP");
    delay(1000);
  }
}
```
#### Using original [WIZnet-ArduinoEthernet](https://github.com/WIZnet-ArduinoEthernet/Ethernet)
```
#include <Ethernet.h>
//#include <EventEthernet.h> // note: you can use EventEthernet lib on existing code with polling method 
void setup() {
  Ethernet.init(17); // W5100S-EVB-Pico: nCS pin = GPIO17
  while (Ethernet.begin(mac) == 0)
  {
    LOG_DEBUG("Failed to configure Ethernet using DHCP");
    delay(1000);
  }
}
```

---

## Why do we need this [EventEthernet library](https://github.com/teamprof/arduino-eventethernet)

### Features

This library is to facilitate your usage of W5100S-EVB-Pico board with the onboard Wiznet W5100S chip, using [**Arduino-mbed RP2040** core](https://github.com/arduino/ArduinoCore-mbed)

In case you are looking for an event driven approach of receiving Ethernet data/status, rathan than the traditional polling method, this library may be one of choices for you.

```
Events to be dispatched to: void (*EthernetEventCallback)(uint8_t ir, uint8_t ir2, uint8_t slir);
1. Interrupt Register (IR) 
     EthernetEventApi::Ir::CONFLICT
     EthernetEventApi::Ir::UNREACH
     EthernetEventApi::Ir::PPPTERM
   note: NO Sn_INT events in EthernetEventCallback. User should use Sn_IR in SocketEventCallback

2. Interrupt Register 2 (IR2)
     EthernetEventApi::Ir2::WOL

3. SOCKET-less Interrupt Register (SLIR) 
     EthernetEventApi::Slir::TIMEOUT
     EthernetEventApi::Slir::ARP
     EthernetEventApi::Slir::PING
  
Events to be dispatched to: void (*SocketEventCallback)(uint8_t sn_ir);
1. SOCKET n Interrupt Register (Sn_IR)
    SnIR::SEND_OK
    SnIR::TIMEOUT
    SnIR::RECV
    SnIR::DISCON
    SnIR::CON
```
---

## Currently supported Boards

1. [**W5100S-EVB-Pico**](https://www.wiznet.hk/en/open-source-hardware/114-w5100s-evb-pico.html), using [**Arduino-mbed RP2040** core](https://github.com/arduino/ArduinoCore-mbed)

---

## Prerequisites

1. [`Arduino IDE 1.8.19+` for Arduino](https://www.arduino.cc/en/Main/Software)
2. [`Arduino mbed_rp2040 core 2.7.2-`](https://github.com/arduino/ArduinoCore-mbed) for [W5100S-EVB-Pico](https://www.wiznet.hk/en/open-source-hardware/114-w5100s-evb-pico.html)
3. [Netcat Command in Linux](https://www.linuxfordevices.com/tutorials/netcat-command-in-linux)

---

## Installation

1. Navigate to [**EventEthernet**](https://github.com/teamprof/arduino-eventethernet) page.
2. Download the latest release `EventEthernet.zip`.
3. Extract the zip file to `EventEthernet` directory 
4. Copy whole `EventEthernet` folder to Arduino libraries' directory such as `~/Arduino/libraries/`.


---


## Code Examples 

[EventEthernetDemo](examples/EventEthernetDemo)

1. Receive Ethernet interrupt/status events
```
#define PIN_ETH_CS 17u   // W5100S-EVB-Pico: nCS = GPIO17
#define PIN_ETH_INTN 21u // W5100S-EVB-Pico: INTn = GPIO21

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
static byte mac[] = {
    0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

static void onEthernetEvent(uint8_t ir, uint8_t ir2, uint8_t slir)
{
  LOG_DEBUG("ir = ", DebugLogBase::HEX, ir, "(hex), ir2 = ", DebugLogBase::HEX, ir2, "(hex), slir = ", DebugLogBase::HEX, slir, " (hex)");

  // Interrupt Register event
  if (ir & IR::CONFLICT)
  {
    LOG_DEBUG("Ir::CONFLICT");
  }
  if (ir & IR::UNREACH)
  {
    LOG_DEBUG("Ir::UNREACH");
  }
  if (ir & IR::PPPTERM)
  {
    LOG_DEBUG("Ir::PPPTERM");
  }

  // Interrupt Register 2 event
  if (ir2 & IR2::WOL)
  {
    LOG_DEBUG("Ir2::WOL");
  }

  // SOCKET-less Interrupt Register event
  if (slir & SLIR::TIMEOUT)
  {
    LOG_DEBUG("Slir::TIMEOUT");
  }
  if (slir & SLIR::ARP)
  {
    LOG_DEBUG("Slir::ARP");
  }
  if (slir & SLIR::PING)
  {
    LOG_DEBUG("Slir::PING");
  }
}

static void initNetwork(void)
{
  LOG_DEBUG("Ethernet.init(", PIN_ETH_CS, ", ", PIN_ETH_INTN, ") // W5100S-EVB-Pico: nCS pin = ", PIN_ETH_CS, ", Intn pin = ", PIN_ETH_INTN);
  Ethernet.init(PIN_ETH_CS, PIN_ETH_INTN); // W5100S-EVB-Pico: nCS pin = GPIO17, Intn pin = GPIO21

  // subscribe CONFLICT, UNREACH and PPPTERM interrupts on IR (Interrupt Register)
  // subscribe WOL interrupt on IR2 (Interrupt Register 2)
  // subscribe TIMEOUT, ARP and PING interrupts on SLIR (SOCKET-less Interrupt Register)
  uint8_t ir = IR::CONFLICT | IR::UNREACH | IR::PPPTERM;
  uint8_t ir2 = IR2::WOL;
  uint8_t slir = SLIR::TIMEOUT | SLIR::ARP | SLIR::PING;
  while (Ethernet.begin(mac, ir, ir2, slir, onEthernetEvent) == 0)
  {
    LOG_DEBUG("Failed to configure Ethernet using DHCP");

    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
      LOG_ERROR("Ethernet was not found. Sorry, can't run without hardware. :(");
    }
    else if (Ethernet.linkStatus() == LinkOFF)
    {
      LOG_DEBUG("Ethernet cable is not connected");
    }

    delay(1000);
  }

  LOG_DEBUG("localIP(): ", Ethernet.localIP());
  LOG_DEBUG("dnsServerIP(): ", Ethernet.dnsServerIP());
}

static void setupEventEthernetDemo(void)
{
  initNetwork();
  // ...
}

void setup()
{
  // ...
  setupEventEthernetDemo();
}

```

2. Open an UDP listener to receive socket event on port 8060.
```
static unsigned int udpPort = 8060;
static EventEthernetUDP udp;

static const int UDP_RX_PACKET_SIZE = 512;
static const int UDP_TX_PACKET_SIZE = 512;
static char packetBufferUdpRx[UDP_RX_PACKET_SIZE];
static char packetBufferUdpTx[UDP_TX_PACKET_SIZE];

static void readUdpPacket(void)
{
  // ...
}
static void writeUdpPacket(IPAddress ipAddress, int port, const char *data)
{
  // ...
}

static void onUdpEvent(uint8_t sr_ir)
{
  if (sr_ir & SnIR::SEND_OK)
  {
    LOG_DEBUG("SnIR::SEND_OK");
  }
  if (sr_ir & SnIR::TIMEOUT)
  {
    LOG_DEBUG("SnIR::TIMEOUT");
  }
  if (sr_ir & SnIR::RECV)
  {
    LOG_DEBUG("SnIR::RECV");
    readUdpPacket();

    memset(packetBufferUdpTx, 0, UDP_RX_PACKET_SIZE);
    sprintf(packetBufferUdpTx, "echo received data: %s", packetBufferUdpRx);
    writeUdpPacket(udp.remoteIP(), udp.remotePort(), packetBufferUdpTx);

    static int count = 1;
    sprintf(packetBufferUdpTx, "Received %d packet(s), content = %s", count++, packetBufferUdpRx);
    writeUdpPacket(udp.remoteIP(), udpPort, packetBufferUdpTx);
  }
  if (sr_ir & SnIR::DISCON)
  {
    LOG_DEBUG("SnIR::DISCON");
  }
  if (sr_ir & SnIR::CON)
  {
    LOG_DEBUG("SnIR::CON");
  }
}

static void initUdpEvent(void)
{
  uint8_t sn_ir = SnIR::SEND_OK | SnIR::TIMEOUT | SnIR::RECV | SnIR::DISCON | SnIR::CON;

  udp.begin(udpPort, sn_ir, onUdpEvent);
  LOG_DEBUG("Listening UDP on ", Ethernet.localIP(), ":", udpPort);
}

static void setupEventEthernetDemo(void)
{
  initNetwork();
  initUdpEvent();
  //...
}

void setup() 
{
  // ...
  setupEventEthernetDemo();
}
```

3. Open a TCP client, connect to a TCP socket listener, send data via traditional Ethernet lib and receive data event via socket event.
```
static const char tcpClientIP[] = "192.168.0.171";  // change it to PC's IP address
static const int tcpClientPort = 8080;

static const int TCP_RX_BUFFER_SIZE = 1024;
static uint8_t bufferTcpRx[TCP_RX_BUFFER_SIZE];

static void readTcpPacket(void)
{
  while (true)
  {
    int tcpSize = tcpClient.available();
    if (tcpSize > 0)
    {
      if (tcpSize > sizeof(bufferTcpRx))
      {
        tcpSize = sizeof(bufferTcpRx) - 1;
      }
      tcpClient.read(bufferTcpRx, tcpSize);
      bufferTcpRx[tcpSize] = 0;
      LOG_DEBUG("Received ", tcpSize, " bytes from ", tcpClient.remoteIP(), ":", tcpClient.remotePort());
      LOG_DEBUG("Content: ", (const char *)bufferTcpRx);
    }
    else
    {
      break;
    }
  }
}

static void onTcpClientEvent(uint8_t sr_ir)
{
  if (sr_ir & SnIR::SEND_OK)
  {
    LOG_DEBUG("SnIR::SEND_OK");
  }
  if (sr_ir & SnIR::TIMEOUT)
  {
    LOG_DEBUG("SnIR::TIMEOUT");
  }
  if (sr_ir & SnIR::RECV)
  {
    LOG_DEBUG("SnIR::RECV");
    readTcpPacket();
  }
  if (sr_ir & SnIR::DISCON)
  {
    LOG_DEBUG("SnIR::DISCON");
  }
  if (sr_ir & SnIR::CON)
  {
    LOG_DEBUG("SnIR::CON");
  }
}

static void initTcpClientEvent(void)
{
  // close previous opened socket
  tcpClient.stop();

  uint8_t sn_ir = SnIR::SEND_OK | SnIR::TIMEOUT | SnIR::RECV | SnIR::DISCON | SnIR::CON;
  if (tcpClient.connect(tcpClientIP, tcpClientPort, sn_ir, onTcpClientEvent))
  {
    LOG_DEBUG("Connected to ", tcpClient.remoteIP(), ":", tcpClient.remotePort());
  }
  else
  {
    // if you didn't get a connection to the server:
    LOG_DEBUG("Connected to telnet ", tcpClientIP, ":", tcpClientPort, " failed!");
  }
}

static void setupEventEthernetDemo(void)
{
  initNetwork();
  // ...
  initTcpClientEvent();
}

void setup() 
{
  // ...
  setupEventEthernetDemo();
}
```

4. Open a TCP server to act as a simple web server, response to client via socket event
```
static uint16_t tcpServerPort = 80;
static EventEthernetServer tcpServer(tcpServerPort);

static void handleNewConnectionEvent(void)
{
  // ...
}

static void onTcpServerEvent(uint8_t sr_ir)
{
  if (sr_ir & SnIR::SEND_OK)
  {
    LOG_DEBUG("SnIR::SEND_OK");
  }
  if (sr_ir & SnIR::TIMEOUT)
  {
    LOG_DEBUG("SnIR::TIMEOUT");
  }
  if (sr_ir & SnIR::RECV)
  {
    LOG_DEBUG("SnIR::RECV");
  }
  if (sr_ir & SnIR::DISCON)
  {
    LOG_DEBUG("SnIR::DISCON");
  }
  if (sr_ir & SnIR::CON)
  {
    LOG_DEBUG("SnIR::CON");
    handleNewConnectionEvent();
  }
}

static void initWebServer(void)
{
  uint8_t sn_ir = SnIR::SEND_OK | SnIR::TIMEOUT | SnIR::RECV | SnIR::DISCON | SnIR::CON;
  tcpServer.begin(sn_ir, onTcpServerEvent);
  LOG_DEBUG("Simple Web server at: ", Ethernet.localIP(), ":", tcpServerPort);
}

static void setupEventEthernetDemo(void)
{
  initNetwork();
  // ...
  initWebServer();
  // ...
}

void setup() 
{
  // ...
  setupEventEthernetDemo();
}
```
Please refer to [EventEthernetDemo] for complete code

---

## Demos

Before running the demo example in Arduino IDE, launch couples of Linux terminals and start UDP and TCP listeners on PC.

On terminal 1, type "nc -u -l 8060" <enter> (without quotation mark) to listen UDP data on the Linux PC's port 8060

On terminal 2, type "nc -l 8080" <enter> to listen TCP on the Linux PC's port 8080.

Connect the [W5100S-EVB-Pico] to PC with an USB cable. Plug an Ethernet cable to [W5100S-EVB-Pico].

Launch Arduino IDE, run the the demo example [EventEthernetDemo](examples/EventEthernetDemo). 
Launch Arduino serial monitor and it should show the following output 
```
current log level is 5
[DEBUG] EventEthernetDemo.ino L.345 initNetwork : Ethernet.init( 17 ,  21 ) // W5100S-EVB-Pico: nCS pin =  17 , Intn pin =  21
[DEBUG] EventEthernetDemo.ino L.372 initNetwork : localIP():  192.168.0.126
[DEBUG] EventEthernetDemo.ino L.373 initNetwork : dnsServerIP():  192.168.0.1
[DEBUG] EventEthernetDemo.ino L.294 initUdpEvent : Listening UDP on  192.168.0.126 : 8060
[DEBUG] EventEthernetDemo.ino L.111 initWebServer : Simple Web server at:  192.168.0.126 : 80
[DEBUG] EventEthernetDemo.ino L.218 initTcpClientEvent : Connected to 192.168.0.171 : 8080
[DEBUG] EventEthernetDemo.ino L.210 onTcpClientEvent : SnIR::CON



<<< launch a browser on PC and visit 192.168.0.126, the Arduino Serial Monitor should display >>>
[DEBUG] EventEthernetDemo.ino L.94 onTcpServerEvent : SnIR::RECV
[DEBUG] EventEthernetDemo.ino L.102 onTcpServerEvent : SnIR::CON
[DEBUG] EventEthernetDemo.ino L.36 handleNewConnectionEvent : new client from  192.168.0.171
[DEBUG] EventEthernetDemo.ino L.94 onTcpServerEvent : SnIR::RECV
[DEBUG] EventEthernetDemo.ino L.98 onTcpServerEvent : SnIR::DISCON

<<< launch the third Linux terminal 3, type "nc -u 192.168.0.126 8060" <enter>, it connects to [W5100S-EVB-Pico] UDP listener. >>>
<<< On terminal 3, type "hello from terminal" <enter> >>>
<<< terminal 3 should show: >>>
hello from terminal
echo received data: hello from terminal

<<< Linux terminal 1 should show: >>>
Received 1 packet(s), content = hello from terminal

<<< The Arduino Serial Monitor should show: >>>
[DEBUG] EventEthernetDemo.ino L.269 onUdpEvent : SnIR::RECV
[DEBUG] EventEthernetDemo.ino L.240 readUdpPacket : Received  20  bytes from  192.168.0.171 : 45974
[DEBUG] EventEthernetDemo.ino L.250 writeUdpPacket : Address =  192.168.0.171 : 45974 , data =  echo received data: hello from terminal
[DEBUG] EventEthernetDemo.ino L.250 writeUdpPacket : Address =  192.168.0.171 : 8060 , data =  Received 1 packet(s), content = hello from terminal

<<<  On terminal 2, type hey <enter> >>>
<<< terminal 2 should show: >>>
  hey
  
<<< Arduino Serial Monitor should show: >>>
  [DEBUG] EventEthernetDemo.ino L.80 onTcpClientEvent : SnIR::RECV
  [DEBUG] EventEthernetDemo.ino L.58 readTcpPacket : Received 4 bytes from 192.168.0.171 : 8080
  [DEBUG] EventEthernetDemo.ino L.59 readTcpPacket : Content: hey

<<< On Arduino Serial Monitor, type "hello" <control+enter> >>>
<<< terminal 2 should show: >>>
  hey
  hello

```

## Debug

Debug is disabled by "#define DEBUGLOG_DISABLE_LOG"

Enable trace debug by "#define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE"

Example of EventEthernetDemo.ino
```
// Disable Logging Macro (Release Mode)
// #define DEBUGLOG_DISABLE_LOG

// You can also set default log level by defining macro (default: INFO)
#define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE
#include <DebugLog.h> // https://github.com/hideakitai/DebugLog

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(100);
  }
  LOG_ATTACH_SERIAL(Serial);

  // The default log_leval is DebugLogLevel::LVL_INFO
  // 0: NONE, 1: ERROR, 2: WARN, 3: INFO, 4: DEBUG, 5: TRACE
  PRINTLN("current log level is", (int)LOG_GET_LEVEL());

  // ...
}
```
## Screenshots

[![screenshot-arduino](/doc/screenshot-arduino.jpg)](https://github.com/teamprof/arduino-eventethernet/blob/main/doc/screenshot-arduino.jpg)

[![screenshot-terminals](/doc/screenshot-terminals.jpg)](https://github.com/teamprof/arduino-eventethernet/blob/main/doc/screenshot-terminals.jpg)

---

## Troubleshooting

If you get compilation errors, more often than not, you may need to install a newer version of the core for Arduino boards.

Sometimes, the library will only work if you update the board core to the latest version because I am using newly added functions.

---

## Issues

Submit issues to: [EventEthernet_Mbed_RP2040 issues](https://github.com/teamprof/arduino-eventethernet/issues) 

---

## TO DO

1. Search for bug and improvement.

---

## DONE

1. Basic EventEthernet for **W5100S-EVB-Pico board**, using [**Arduino-mbed RP2040** core](https://github.com/arduino/ArduinoCore-mbed)

---
---

## Contributions and Thanks

Many thanks for everyone for bug reporting, new feature suggesting, testing and contributing to the development of this library.

---

## Contributing

If you want to contribute to this project:

- Report bugs and errors
- Ask for enhancements
- Create issues and pull requests
- Tell other people about this library

---

## License

- The library is licensed under GNU LESSER GENERAL PUBLIC LICENSE Version 3
---

## Copyright

- Copyright 2022- teamprof.net@gmail.com

