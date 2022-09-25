#include "trj_types.h"

class IMessageProcessor {

    public:

    IMessageProcessor();
    virtual ~IMessageProcessor() = default;

public: 


  virtual  void update() = 0;

  virtual void setLastSegNum(int v) = 0;

  virtual void sendAck(uint16_t seq, CurrentState &current_state) = 0;

  // An ACK with no current state
  virtual void sendAck(uint16_t seq) = 0;

  virtual void sendNack() = 0;

  virtual void sendDone(uint16_t seq, CurrentState &current_state) = 0;

  virtual void sendEmpty(uint16_t seq, CurrentState &current_state) = 0;
 
  // Send a text message
  virtual void sendMessage(const char *message_) = 0;

  virtual void printf(const char* fmt, ...) = 0;

  // Remove a message from the queue
  virtual Message popMessage() = 0;

  virtual int availableMessages() = 0;

};