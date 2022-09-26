#pragma once

#include <PacketSerial.h>
#include "trj_const.h"
#include "imessageprocessor.h"
#include <functional>
#include <deque>
#include <vector>

#include "trj_types.h"


using namespace std;

using BufferPtr = const uint8_t*;

class MessageProcessor :  public virtual IMessageProcessor {

private:

  IPacketSerial &ps;

  uint8_t buffer[MESSAGE_BUF_SIZE]; // Outgoing message buffer

  int lastSegNum=0;

  std::deque<Message> messages;

public: 

  MessageProcessor(IPacketSerial &ps);

  void update() override;

  void updateAll();

  void setLastSegNum(int v) override;

  void sendAck(uint16_t seq, CurrentState &current_state) override;

  // An ACK with no current state
  void sendAck(uint16_t seq) override;

  void sendNack() override;

  void sendDone(uint16_t seq, CurrentState &current_state) override;

  void sendEmpty(uint16_t seq, CurrentState &current_state) override;
 
  // Send a text message
  void sendMessage(const char *message_) override;

  void printf(const char* fmt, ...) override;

  // Remove a message from the queue
  void pop() override;

  Message& firstMessage() override;

  int availableMessages() override;

  bool empty() override;

  // Perform operation for each type of message recieved
  void processPacket(const uint8_t* buffer_, size_t size) override;

  void processPacket(PacketHeader *ph, const uint8_t *payload, size_t payload_size) override;

  void processPacket(PacketHeader ph, char *payload, size_t payload_size);

private:

  uint8_t crc(size_t length);

  void send(size_t length);

  void send(CommandCode code, uint16_t seq, size_t length);

  void send(const uint8_t* payload, CommandCode code, uint16_t seq, size_t length);

};
