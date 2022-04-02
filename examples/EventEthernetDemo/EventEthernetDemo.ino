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
// demo TCP client event by connecting it to a Linux PC TCP listener
// Launch a Linux terminal (terminal 2) on PC
// On terminal 2, type "nc -l 8080" <enter> to listen TCP on the Linux PC's port 8080.
// On Arduino IDE, "upload" the demo, launch "Serial Monitor" and search initTcpClientEvent
// Here is an example of the log
//   [DEBUG] EventEthernetDemo.ino L.218 initTcpClientEvent : Connected to 192.168.0.171 : 8080
// On Arduino Serial Monitor, type hello <Control + Enter>
// terminal 2 should show:
//   hello
// On terminal 2, type hey <enter>
// Arduino Serial Monitor should show:
//  [DEBUG] EventEthernetDemo.ino L.200 onTcpClientEvent : SnIR::RECV
//  [DEBUG] EventEthernetDemo.ino L.166 readTcpPacket : Received 4 bytes from 192.168.0.171 : 8080
//  [DEBUG] EventEthernetDemo.ino L.167 readTcpPacket : Content: hey
///////////////////////////////////////////////////////////////////////////////
static const char tcpClientIP[] = "192.168.0.171"; // Linux PC IP address
static const int tcpClientPort = 8080;

static EventEthernetClient tcpClient; // demo of TCP client

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

static void deinitTcpClientEvent(void)
{
  tcpClient.stop();
}

///////////////////////////////////////////////////////////////////////////////
// demo TCP server event by starting a simple Web server
// server port: 80
// On Arduino IDE, "upload" the demo, launch "Serial Monitor" and search the Web server IP:port
// Here is an example of the log
//   [DEBUG] EventEthernetDemo.ino  initTcpServerEvent : Simple Web server at: 192.168.0.126 : 80
// Launch a web browser (Chrome/Edge/...), type "http:// 192.168.0.126" on the address bar, and press "Enter"
// The web browser should show "Hello EventEthernetDemo"
///////////////////////////////////////////////////////////////////////////////
static uint16_t tcpServerPort = 80;
static EventEthernetServer tcpServer(tcpServerPort);

static void handleNewConnectionEvent(void)
{
  EventEthernetClient client = tcpServer.accept();
  // EventEthernetClient client = tcpServer.available();
  if (client)
  {
    LOG_DEBUG("new client from ", client.remoteIP());

    bool currentLineIsBlank = true; // an http request ends with a blank line
    while (client.connected())
    {
      if (client.available())
      {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank)
        {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close"); // the connection will be closed after completion of the response
          client.println("Refresh: 5");        // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");

          client.println("Hello EventEthernetDemo");
          client.println("<br />");

          client.println("</html>");
          break;
        }
        if (c == '\n')
        {
          currentLineIsBlank = true; // you're starting a new line
        }
        else if (c != '\r')
        {
          currentLineIsBlank = false; // you've gotten a character on the current line
        }
      }
    }

    delay(100);    // give the web browser time to receive the data
    client.stop(); // close the connection
    // LOG_DEBUG("client disconnected");
  }
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

///////////////////////////////////////////////////////////////////////////////
// demo Event UDP
// Launch a Linux terminal (terminal 1) on PC, and type the following command to listen UDP data on the Linux PC's port 8060
//   nc -u -l 8060 <enter>
// On Arduino IDE, "upload" the demo, launch "Serial Monitor" and search the localIP()
// Here is an example of the log
//   [DEBUG] EventEthernetDemo.ino L.288 initNetwork : localIP(): 192.168.0.126
// Launch a Linux terminal (terminal 3) on PC, type the following command to connect [W5100S-EVB-Pico]'s UDP listener
//   nc -u 192.168.0.126 8060 <enter>
// On terminal 3, type "hello" <enter>, terminal 3 should show:
//   hello
//   echo received data: hello
// terminal 1 should show the followings
//   Received 1 packet(s), content = hello
///////////////////////////////////////////////////////////////////////////////
static unsigned int udpPort = 8060;
static EventEthernetUDP udp;

static const int UDP_RX_PACKET_SIZE = 512;
static const int UDP_TX_PACKET_SIZE = 512;
static char packetBufferUdpRx[UDP_RX_PACKET_SIZE];
static char packetBufferUdpTx[UDP_TX_PACKET_SIZE];

static void readUdpPacket(void)
{
  int packetSize = udp.parsePacket();
  if (packetSize)
  {
    // if (udpListen.available() && udpListen.parsePacket()) {
    LOG_DEBUG("Received ", packetSize, " bytes from ", udp.remoteIP(), ":", udp.remotePort());

    udp.read(packetBufferUdpRx, packetSize);
    // LOG_DEBUG("Content: ", packetBufferUdpRx);
    // LOG_DEBUG("Content: ", packetBufferUdpRx, packetSize);
    // LOG_DEBUG("Content: ", LOG_AS_ARR(packetBufferUdpRx, packetSize - 1));
  }
}
void writeUdpPacket(IPAddress ipAddress, int port, const char *data)
{
  LOG_DEBUG("Address = ", ipAddress, ":", port, ", data = ", data);

  udp.beginPacket(ipAddress, port);
  udp.write(data, strlen(data) + 1);
  udp.endPacket();
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
static void deinitUdpEvent(void)
{
  udp.stop();
}

///////////////////////////////////////////////////////////////////////////////
// demo Ethenet Event
///////////////////////////////////////////////////////////////////////////////
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
  initUdpEvent();
  initWebServer();
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

#define POLL_EXTERNAL_WEB_INTERVAL 10 // 30s, interval of sending http request to external web server
static int timerWebRequest = 0;

void loop()
{
  delay(5000);
}
