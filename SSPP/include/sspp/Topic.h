#ifndef SSPP_TOPIC_H
#define SSPP_TOPIC_H

#include "Service.h"
#include "Config.h"

namespace SSPP {
  using namespace std;

/** @brief A base class for reading from/writing to a topic server.  Usually
 * you will use TopicListener, TopicValue, MultiTopicListener, or
 * MultiTopicValue rather than this class.
 *
 * When Subscribe is called, the server will start sending messages on that
 * topic.  If Subscribe is called multiple times, the server will send
 * wrapped messages, with the structure {"path":[topic],"data":[message body}.
 *
 * When Get is called, the server will send a single message back on that
 * topic.  Be careful not to mix Get and Subscribe calls.
 *
 * Set completely changes the topic's data on the server.
 *
 * Change only changes the sub topics specified in the message.
 *
 * Resize instantiates an empty array of the given size on the specified topic.
 *
 * Delete removes a topic from the server.
 */
class TopicClient : public Service
{
 public:
  TopicClient(const char* host=SSPP_DEFAULT_TOPIC_SERVER,double timeout=0);
  virtual ~TopicClient() {}
  virtual const char* Name() const { return "TopicClient"; }
  virtual const char* Description() const { return "Connects to a topic server"; }
  void Subscribe(const string& topic,double rate=0);
  void Unsubscribe(const string& topic);
  void Set(const string& topic,const AnyCollection& data);
  void Get(const string& topic);
  void Change(const string& topic,const AnyCollection& data);
  void Resize(const string& topic,size_t n);
  void Delete(const string& topic);
  //Override this to handle messages from subscribe / get for all topics
  virtual bool OnMessage(AnyCollection& message);
  //Override this to handle messages from subscribe / get
  virtual bool OnTopicMessage(const string& topic,AnyCollection& message) { return true; }

  ///if only one topic is subscribed to, a topic server is only going to send one
  string firstTopic;
};

/** @brief A class that subscribes to a topic on a topic server. 
 * Need to run Process on this service (e.g., runwhile, etc)
 */
class TopicListener : public TopicClient
{
 public:
  TopicListener(const char* host=SSPP_DEFAULT_TOPIC_SERVER,const char* topic=".",double timeout=0);
  virtual ~TopicListener() {}
  virtual const char* Name() const { return "TopicListener"; }
  virtual const char* Description() const { return "Listens to a topic on a topic server"; }
  virtual bool OnMessage(AnyCollection& message);
  ///Returns the latest value, and automatically marks the value as being
  ///read.
  AnyCollection& Get();
  ///Returns the number of messages received since the last Get() or
  ///MarkRead() call.
  int UnreadCount() const { return unreadMessageCount; }
  ///Manually marks the value as being read.
  void MarkRead() { unreadMessageCount = 0; }

  AnyCollection value;
  int messageCount,unreadMessageCount;
};

/** @brief A class that retrieves / changes topics on a topic server. 
 * Do not need to run Process.
 */
class TopicValue : public TopicClient
{
 public:
  TopicValue(const char* host=SSPP_DEFAULT_TOPIC_SERVER,const char* topic=".",double timeout=0);
  virtual ~TopicValue() {}
  virtual const char* Name() const { return "TopicValue"; }
  virtual const char* Description() const { return "Reads/writes to a topic on a topic server"; }
  virtual bool OnMessage(AnyCollection& message);
  AnyCollection& Get(double timeout=Math::Inf);
  void Set();
  void Set(const AnyCollection& value);

  string topic;
  AnyCollection value;
};

/** @brief A class that subscribes to multiple topics on a topic server. 
 * Need to run Process on this service (e.g., runwhile, etc)
 */
class MultiTopicListener : public TopicClient
{
 public:
  MultiTopicListener(const char* host=SSPP_DEFAULT_TOPIC_SERVER,double timeout=0);
  virtual ~MultiTopicListener() {}
  AnyCollection& Listen(const string& topic);
  virtual const char* Name() const { return "MultiTopicListener"; }
  virtual const char* Description() const { return "Listens to multiple topics on a topic server"; }
  virtual bool OnTopicMessage(const string& topic,AnyCollection& message);
  AnyCollection& Get(const string& topic);
  int UnreadCount(const string& topic) const;
  void MarkRead(const string& topic);

  struct ValueInfo
  {
    AnyCollection value;
    int messageCount,unreadMessageCount;
  };
  map<string,ValueInfo> values;
};

/** @brief A class that gets/sets multiple topics on a topic server. 
 * Do not need to run Process on this service.
 */
class MultiTopicValue : public TopicClient
{
 public:
  MultiTopicValue(const char* host=SSPP_DEFAULT_TOPIC_SERVER,double timeout=0);
  virtual ~MultiTopicValue() {}
  virtual const char* Name() const { return "MultiTopicValue"; }
  virtual const char* Description() const { return "Reads/writes to multiple topics on a topic server"; }
  virtual bool OnTopicMessage(const string& topic,AnyCollection& message);
  AnyCollection& Get(const string& topic,double timeout=Math::Inf);
  void Set(const string& topic,const AnyCollection& value);

  map<string,AnyCollection> values;
};


///Convenience function: connects to a service and requests a topic.  Returns
///an empty collection on error.
AnyCollection ReadTopic(const char* addr=SSPP_DEFAULT_TOPIC_SERVER,const char* topic=".",double timeout=0);

} //namespace SSPP

#endif 
