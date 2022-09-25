#pragma once

#include <PacketSerial.h>
#include "trj_const.h"
#include "imessageprocessor.h"
#include <functional>
#include <deque>
#include <vector>

#include "trj_types.h"

//#include "trj_ringbuffer.h"
//#include "trj_sdstepper.h"
//#include "trj_stepdriver.h"

using namespace std;


class MessageProcessor : public PacketSerial, public IMessageProcessor {

private:

  Stream& stream;

  uint8_t buffer[MESSAGE_BUF_SIZE]; // Outgoing message buffer

  int lastSegNum=0;

  std::deque<Message> messages;

public: 

  MessageProcessor(Stream &stream);

  void update() override;

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
  Message popMessage() override;

  int availableMessages() override;

private:

  uint8_t crc(size_t length);

  // Perform operation for each type of message recieved
  void processPacket(const uint8_t* buffer_, size_t size);

  void send(size_t length);

  void send(CommandCode code, uint16_t seq, size_t length);

  void send(const uint8_t* buffer_, CommandCode code, uint16_t seq, size_t length);

};
