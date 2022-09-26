#pragma once
#include <functional>
#include <deque>
#include <vector>

#include "trj_const.h"
#include "trj_types.h"

#include "ipacketserial.h"

using namespace std;

using BufferPtr = const uint8_t*;

class MessageProcessor  {

private:

  IPacketSerial &ps;

  uint8_t buffer[MESSAGE_BUF_SIZE]; // Outgoing message buffer

  int lastSegNum=0;

  std::deque<Message> messages;

public: 

  MessageProcessor(IPacketSerial &ps);

  void update();

  void updateAll();

  void setLastSegNum(int v);

  void sendAck(uint16_t seq, CurrentState &current_state);

  // An ACK with no current state
  void sendAck(uint16_t seq);

  void sendNack();

  void sendDone(uint16_t seq, CurrentState &current_state);

  void sendEmpty(uint16_t seq, CurrentState &current_state);
 
  // Send a text message
  void sendMessage(const char *message_);

  void printf(const char* fmt, ...);

  // Remove a message from the queue
  void pop();

  Message& firstMessage();

  int availableMessages();

  bool empty();

  // Perform operation for each type of message recieved
  void processPacket(const uint8_t* buffer_, size_t size);

  void processPacket(PacketHeader *ph, const uint8_t *payload, size_t payload_size);

  void processPacket(PacketHeader ph, char *payload, size_t payload_size);

private:

  uint8_t crc(size_t length);

  void send(size_t length);

  void send(CommandCode code, uint16_t seq, size_t length);

  void send(const uint8_t* payload, CommandCode code, uint16_t seq, size_t length);

};
