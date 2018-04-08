#include <sspp/Topic.h>
#include <sspp/Config.h>

namespace SSPP {

TopicClient::TopicClient(const char* host,double timeout) 
{ 
  if(!OpenClient(host,timeout)) {
    printf("Waiting for host %s to open\n",host);
  }
}
void TopicClient::Subscribe(const string& topic,double rate) 
{
  if(firstTopic.empty()) firstTopic = topic;
  AnyCollection msg;
  msg["type"] = string("subscribe");
  msg["path"] = topic;
  if(rate!=0)
    msg["rate"] = rate;
  SendMessage(msg);
}

void TopicClient::Unsubscribe(const string& topic) 
{
  //TODO: danger of switching from a multi-subscribe to a single-subscribe
  if(!firstTopic.empty()) firstTopic.clear();
  AnyCollection msg;
  msg["type"] = string("unsubscribe");
  msg["path"] = topic;
  SendMessage(msg);
}

void TopicClient::Set(const string& topic,const AnyCollection& data) 
{
  AnyCollection msg;
  msg["type"] = string("set");
  msg["path"] = topic;
  msg["data"] = data;
  SendMessage(msg);
}

void TopicClient::Get(const string& topic) 
{
  if(firstTopic.empty()) firstTopic = topic;
  AnyCollection msg;
  msg["type"] = string("get");
  msg["path"] = topic;
  SendMessage(msg);
}

void TopicClient::Change(const string& topic,const AnyCollection& data) 
{
  AnyCollection msg;
  msg["type"] = string("change");
  msg["path"] = topic;
  msg["data"] = data;
  SendMessage(msg);
}

void TopicClient::Resize(const string& topic,size_t n) 
{
  AnyCollection msg;
  msg["type"] = string("resize");
  msg["path"] = topic;
  msg["data"] = int(n);
  SendMessage(msg);
}

void TopicClient::Delete(const string& topic) 
{
  AnyCollection msg;
  msg["type"] = string("delete");
  msg["path"] = topic;
  SendMessage(msg);
}

bool TopicClient::OnMessage(AnyCollection& message)
{
  if(message.collection() && message.find("path")) {
    string topic;
    if(!message["path"].as(topic)) {
      fprintf(stderr,"TopicClient: path item is not a string?\n");
      return false;
    }
    return OnTopicMessage(topic,message["data"]);
  }
  if(firstTopic.empty()) {
    fprintf(stderr,"TopicClient: got a message without requesting anything?\n");
    return false;
  }
  return OnTopicMessage(firstTopic,message);
}

TopicListener::TopicListener(const char* host,const char* topic,double timeout)
  :TopicClient(host,timeout)
{
  Subscribe(topic);
  messageCount = 0;
  unreadMessageCount = 0;
  //this is helpful for slow readers / fast senders
  onlyProcessNewest=true;
}

bool TopicListener::OnMessage(AnyCollection& message)
{
  value = message;
  messageCount++;
  unreadMessageCount++;
  return true;
}

AnyCollection& TopicListener::Get()
{
  unreadMessageCount = 0;
  return value;
}

TopicValue::TopicValue(const char* host,const char* _topic,double timeout)
  :TopicClient(host,timeout),topic(_topic)
{
  //this is helpful for slow readers / fast senders
  onlyProcessNewest = true;
}

bool TopicValue::OnMessage(AnyCollection& message)
{
  value = message;
  return true;
}

void TopicValue::Set(const AnyCollection& value)
{
  TopicClient::Change(topic,value);
}

void TopicValue::Set()
{
  TopicClient::Change(topic,value);
  TopicClient::Process();
}

AnyCollection& TopicValue::Get(double timeout)
{
  TopicClient::Get(topic);
  Timer timer;
  value.clear();
  while(value.size()==0) {
    int n = Process();
    if(n < 0) {
      printf("TopicValue: error reading while waiting for Get() response\n");
      return value;
    }
    if(value.size()!=0) return value;
    if(!Connected()) {
      printf("TopicValue: was disconnected while waiting for Get() response\n");
      return value;
    }
    if(timer.ElapsedTime() > timeout) {
      printf("TopicValue: timeout reached while waiting for Get() response\n");
      return value;
    }
    ThreadSleep(SSPP_MESSAGE_WAIT_TIME);
  }
  return value;
}

MultiTopicListener::MultiTopicListener(const char* host,double timeout)
  :TopicClient(host,timeout)
{}

AnyCollection& MultiTopicListener::Listen(const string& topic)
{
  if(values.count(topic) == 0)
    TopicClient::Subscribe(topic);
  ValueInfo& v = values[topic];
  v.messageCount = 0;
  v.unreadMessageCount = 0;
  return v.value;
}

bool MultiTopicListener::OnTopicMessage(const string& topic,AnyCollection& message)
{
  ValueInfo& v = values[topic];
  v.value = message;
  v.messageCount ++;
  v.unreadMessageCount ++;
  return true;
}

AnyCollection& MultiTopicListener::Get(const string& topic)
{
  ValueInfo& v = values[topic];
  v.unreadMessageCount = 0;
  return values[topic].value;
}

int MultiTopicListener::UnreadCount(const string& topic) const
{
  return values.find(topic)->second.unreadMessageCount;
}

void MultiTopicListener::MarkRead(const string& topic)
{
  values[topic].unreadMessageCount = 0;
}

MultiTopicValue::MultiTopicValue(const char* host,double timeout)
  :TopicClient(host,timeout)
{}

bool MultiTopicValue::OnTopicMessage(const string& topic,AnyCollection& message)
{
  values[topic] = message;
  return true;
}

AnyCollection& MultiTopicValue::Get(const string& topic,double timeout)
{
  TopicClient::Get(topic);
  Timer timer;
  AnyCollection& value = values[topic];
  value.clear();
  while(value.size()==0) {
    int n = Process();
    if(n < 0) {
      printf("MultiTopicValue: error reading while waiting for Get() response\n");
      return value;
    }
    if(value.size()!=0) return value;
    if(!Connected()) {
      printf("MultiTopicValue: was disconnected while waiting for Get() response\n");
      return value;
    }
    if(timer.ElapsedTime() > timeout) {
      printf("MultiTopicValue: timeout reached while waiting for Get() response\n");
      return value;
    }
    ThreadSleep(SSPP_MESSAGE_WAIT_TIME);
  }
  return value;
}

void MultiTopicValue::Set(const string& topic,const AnyCollection& value)
{
  TopicClient::Change(topic,value);
  TopicClient::Process();
}

class TopicReaderService : public TopicClient
{
public:
  vector<AnyCollection> messages;
  int max;

  TopicReaderService(const char* addr,const char* topic,double timeout,int _max=1)
    :TopicClient(addr,timeout),max(_max)
  {
    Get(topic);
  }
  virtual int Process() {
    int n=TopicClient::Process();
    if((int)messages.size() >= max) return -1;
    return n;
  }
  virtual bool OnMessage(AnyCollection& m) 
  { 
    cout<<"TopicReader: message "<<m<<endl;
    messages.push_back(m); 
    if((int)messages.size() >= max) return false;
    return true; 
  }
};

AnyCollection ReadTopic(const char* addr,const char* topic,double timeout)
{
  printf("ReadTopic...\n");
  SyncPipe pipe;
  pipe.transport = new SocketClientTransport(addr);
  if(!pipe.Start()) {
    printf("ReadTopic: Couldn't connect to %s\n",addr);
    return AnyCollection();
  }
  AnyCollection msg;
  msg["type"] = string("get");
  msg["path"] = string(topic);
  stringstream ss;
  ss<<msg;
  pipe.Send(ss.str());
  Timer timer;
  while(true) {
    if(timeout > 0 && timer.ElapsedTime() >= timeout) break;
    pipe.Work();
    if(pipe.UnreadCount() > 0) {
      stringstream ss(pipe.Newest());
      AnyCollection res;
      ss>>res;
      if(!ss) {
	printf("ReadTopic: incorrectly formatted reply to %s/%s\n",addr,topic);
	return AnyCollection();
      }
      //success
      return res;
    }
    //didn't receive reply yet
    printf("ReadTopic: waiting...\n");
    ThreadSleep(SSPP_MESSAGE_WAIT_TIME);
  }
  printf("ReadTopic: Timeout occurred\n");
  return AnyCollection();
}


} //namespace SSPP
