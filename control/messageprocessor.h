#pragma once
#include <functional>
#include <deque>
#include <vector>
#include <sstream>
#include <string>

#include "trj_const.h"
#include "trj_types.h"

#include "ipacketserial.h"

using namespace std;


class MessageProcessor  {

private:

  IPacketSerial &ps;

  uint8_t buffer[MESSAGE_BUF_SIZE]; // Outgoing message buffer

  int lastSegNum=0;

  std::deque<Message> messages;

  CurrentState current_state;

public: 

  MessageProcessor(IPacketSerial &ps);

  void update(tmillis t, CurrentState &current_state);

  void updateAll(tmillis t, CurrentState &current_state);

  void updateCurrentState(CurrentState &current_state);

  void setLastSegNum(int v);

  void sendAck(uint16_t seq);

  void sendNack();

  void sendDone(uint16_t seq);

  void sendEmpty(uint16_t seq);
 
  // Send a text message
  void sendMessage(const char *message_);
  void sendMessage(const string &str);
  void sendMessage(stringstream &ss);

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

// Singleton message proces for logging on the target.
// THe logging functions will use this mp, if it is set
extern MessageProcessor *message_processor;

void log(const char* str);
void log(const string &str);
void log(stringstream &str);
void log_printf(const char *fmt, ...);


