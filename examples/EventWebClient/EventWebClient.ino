#include <mbed.h>
#include <EventEthernet.h>
#include <utility/w5100.h>

// Disable Logging Macro (Release Mode)
// #define DEBUGLOG_DISABLE_LOG
// You can also set default log level by defining macro (default: INFO)
#define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE
#include <DebugLog.h> // https://github.com/hideakitai/DebugLog

#define PIN_ETH_CS 17u   // W5100S-EVB-Pico: nCS = GPIO17
#define PIN_ETH_INTN 21u // W5100S-EVB-Pico: INTn = GPIO21

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
static byte mac[] = {
    0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

///////////////////////////////////////////////////////////////////////////////
// demo TCP client event by getting response from a simple json server
// Here is an example of the log on Arduino Serial Monitor
//  -> current log level is 5
//  -> [DEBUG] WebClient.ino L.168 initNetwork : Ethernet.init( 17 , 21 ) // W5100S-EVB-Pico: nCS pin = 17 , Intn pin = 21
//  -> [DEBUG] EventEthernet.cpp L.462 subscribeEthernetEvent : this->ir = E0 (hex), this->ir2 = 1 (hex) this->slir = 7 (hex)
//  -> [DEBUG] WebClient.ino L.194 initNetwork : localIP(): 192.168.0.126
//  -> [DEBUG] WebClient.ino L.195 initNetwork : dnsServerIP(): 192.168.0.1
//  -> [DEBUG] EventEthernet.cpp L.55 subscribeSocketEvent : sn_ir = 1F (hex), onSocketEvent = 100030BD (hex), sockindex = 0
//  -> [DEBUG] EventEthernet.cpp L.93 subscribeSocketEvent : this->sn_ir = 1F (hex), this->eventCallback = 100030BD (hex)
//  -> [DEBUG] WebClient.ino L.91 onTcpClientEvent : SnIR::CON
//  -> [DEBUG] WebClient.ino L.101 initTcpClientEvent : Connected to 104.21.4.48 : 80
//  -> [DEBUG] WebClient.ino L.81 onTcpClientEvent : SnIR::RECV
//  -> [DEBUG] WebClient.ino L.47 readTcpPacket : Received 1054 bytes from 104.21.4.48 : 80
//  -> [DEBUG] WebClient.ino L.48 readTcpPacket : Content: HTTP/1.1 200 OK
//  ...
//  -> {
//  -> "userId": 1,
//  -> "id": 1,
//  -> "title": "delectus aut autem",
//  -> "completed": false
//  -> }
//  ...
//  -> [DEBUG] WebClient.ino L.98 onTcpClientEvent : SnIR::DISCON
//  -> [DEBUG] EventEthernet.cpp L.100 unsubscribeSocketEvent : sn_ir = 1F (hex), sockindex = 0
//  -> [DEBUG] EventEthernet.cpp L.142 unsubscribeSocketEvent : this->sn_ir = 0 (hex), this->eventCallback = 0 (hex)

///////////////////////////////////////////////////////////////////////////////
static const char webHost[] = "jsonplaceholder.typicode.com";
static const char webPath[] = "/todos/1";
static const int webPort = 80;

static EventEthernetClient tcpClient; // demo of TCP client

static const int TCP_RX_BUFFER_SIZE = 2048;
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

static void writeTcpPacket(void)
{
  // Make a HTTP request:
  tcpClient.print("GET ");
  tcpClient.print(webPath);
  tcpClient.println(" HTTP/1.1");
  tcpClient.print("Host: ");
  tcpClient.println(webHost);
  tcpClient.println("Connection: close");
  tcpClient.println();
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
    writeTcpPacket();
  }
}

static void initTcpClientEvent(void)
{
  // close previous opened socket
  tcpClient.stop();

  uint8_t sn_ir = SnIR::SEND_OK | SnIR::TIMEOUT | SnIR::RECV | SnIR::DISCON | SnIR::CON;
  if (tcpClient.connect(webHost, webPort, sn_ir, onTcpClientEvent))
  {
    LOG_DEBUG("Connected to ", tcpClient.remoteIP(), ":", tcpClient.remotePort());
  }
  else
  {
    // if you didn't get a connection to the server:
    LOG_DEBUG("Connected to Internet ", webHost, ":", webPort, " failed!");
  }
}

static void deinitTcpClientEvent(void)
{
  tcpClient.stop();
}

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
  initTcpClientEvent();
}

///////////////////////////////////////////////////////////////////////////////
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

  setupEventEthernetDemo();
}

#define POLL_EXTERNAL_WEB_INTERVAL 5 // 10s, interval of sending http request to external web server
// #define POLL_EXTERNAL_WEB_INTERVAL 10 // 10s, interval of sending http request to external web server
static int timerWebRequest = 0;

void loop()
{
  if (timerWebRequest++ >= POLL_EXTERNAL_WEB_INTERVAL)
  {
    timerWebRequest = 0;
    initTcpClientEvent();
  }
  delay(1000);
}