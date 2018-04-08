===============================================================================

  Simple Structure Passing Protocol (SSPP)

  Core Library

  Kris Hauser
  12/8/2014

===============================================================================

These files contain C++/Python convenience classes for the Simple Structure
Passing Protocol and several basic Services for logging, sending, and
printing.


==============================================
  What is SSPP?
==============================================

SSPP is a lightweight, easily implemented, cross-platform messaging protocol
for structured data.  It was designed to fill in some of the deficits of ROS,
while keeping many of its strengths.


==============================================
  SSPP vs other packages
==============================================

Characteristics shared by SSPP and ROS:
+ Structured data
+ Passes messages that can be converted to human-readable format (e.g., JSON)
+ C++ and Python bindings
+ Distributed (ROS Nodes = SSPP Services)

Advantages of SSPP vs ROS:
+ Shallower learning curve
+ No need to conform to a complex build system. 
+ Works on Windows, Android, and any other OS that supports sockets and
  threading
+ Smaller messages
+ Less computational overhead
+ Message are not required to conform to a schema, which makes software
  prototyping faster.

Disadvantages of SSPP vs ROS:
- ROS makes messages conform to a schema, which may catch errors faster.
- ROS communication between nodes is coordinated by a "ROS Master", while
  SSPP requires the user to manage his/her own network topology (e.g.,
  specify IP addresses and ports).
- SSPP doesn't keep connections alive automatically.
- ROS has more debugging tools.
- SSPP does not make any attempt to be a repository for shared modules.
- SSPP has no builtin user base.

Weakesses of both SSPP and ROS, which may be an opportunity for further
development:
- No quality of service guarantees
- No dynamic throttling
- No automatic transport conversions (e.g., encoding, compression, encryption)

Comparison with other related libraries (+ indicates an advantage of SSPP,
- indicates a disadvantage):
+ ZeroMQ: SSPP handles structured data, while ZeroMQ is a raw binary messaging
  system.
- ZeroMQ: supports varied network topologies.  More cross platform libraries
  available (ostensibly -- we have had errors compiling these on some systems).
- Google protocol buffers: messages are binary, which is more compact. 
+ Google protocol buffers: User must use his/her own messaging system.
+ XML: much more verbose.  User must use his/her own messaging system.


==============================================
  How does it work? - Read me before starting!
==============================================

SSPP is a simple protocol: it converts a structured object to a JSON string,
and sends a message containing the string over a socket. The format on the
wire is 4 bytes encoding an integer length of the string, followed by the
string itself. (No null terminating character is sent.)

You are free to use whatever transport protocols you want to send SSPP
messages, but typically standard sockets are used.  A node that sends SSPP
messages over sockets is known as a Service. 

==========
 Services
==========

The SSPP library helps you build your own services painlessly, handling
much of the networking work for you. This involves subclassing the base
Service class and overloading as little as a single function. Using a
this mechanism you can easily send structured messages (SendMessage in C++
and sendMessage in Python) and receive structured messages (OnMessage in
C++ and onMessage in Python).

Services can be set up on the same machine, on the same local network,
or across the Internet.  It is important to note that Services must started
in either client or server mode.  A server can be connected to 0 or more
clients (up to a predetermined limit), while a client is connected to only
a single server.  However, you are free to use as many Services as you
wish in your program.  

==========
 Topics 
==========

To simplify the task of setting up many services that talk to one another,
SSPP also has a "topic" publish/subscribe system.  This somewhat resembles
ROS, except that messages are routed through a server.

To use this service, you start a topic server by running topic_server.py
(structure_service.py is equivalent).  Then you may subscribe using a
C++ TopicService or Python TopicServiceBase, or use the convenience classes
TopicSubscriberBase, TopicValue, or TopicListener.

==========
 Spiffy variables
==========

The Spiffy module helps you conveniently program with remote variables 
almost as though they were native ones.  A client connects to a Spiffy
server, which is run in some separate process.  This is essentially just a 
topic server, with some extra logging / persistence capabilities. 

Once a Spiffy server is launched, you create a Spiffy variable with a given
name.  The variable can now treated more or less like a normal value
via the get() and set() methods.  In C++, you can also get() a value via
casting to the appropriate type, and set() via assignment.  Pretty easy!

Example (C++)

  #include <sspp/Spiffy.h>
  
  int main(int argc,const char** argv) {
    SetSpiffyServer("tcp://[server IP address here]");
    StartSpiffy();
    Spiffy<int> myvar("some_variable_name");
    myvar.set(4); 
    //after set(), all other processes listening to some_variable_name will get
    //the value 4 the next time they call get()
    myvar = 5;  //assignment is an alias for set(5)
    StopSpiffy()
  }

Example (Python)

  from sspp import spiffy

  spiffy.start(([server_ip_address],4567))
  myvar = Spiffy("some_variable_name")
  print myvar.get()  #if you ran the C++ program above first, this would print 5
  spiffy.stop()

==========
 Builtin Services
==========

EchoService: prints data sent to it to the console.
LogService: saves data to disk.  Can also send data from a previously logged
            file.
SendService: sends a message over the command line.
HeartbeatService: sends a heartbeat at a regular rate.
topic_server.py: starts a topic server.
spiffy_server.py: starts a spiffy server.


==============================================
  Programming Guide
==============================================

==========
  C++
==========

=== Building ===

SSPP uses CMake to compile the C++ library / tools.  It also depends on
KrisLibrary. Type

  cmake -DKRISLIBRARY_ROOT=[path to the parent of KrisLibrary] .
  make

to build everything.

The static library libsspp.a (Linux) or libsspp.lib (Windows) will be put
in ./lib while the core servies will be in ./bin.

To install the sspp Python bindings, cd to the Python/ directory and enter

  sudo python setup.py install

You should now be able to load the sspp module in Python using "import sspp".

=== Structure encoding and decoding ===

In C++, you must manually convert your data structures to/from an instance of 
the AnyCollection class. An AnyCollection is a possibly nested structure of
maps, arrays, and primitive values (bool, char, unsigned char, int, unsigned
int, float, double, std::string).  Map keys can be integers, floating point
numbers, or std::strings. 

It's probably easiest to learn by an example.  An AnyCollection can easily
be created dynamically as follows:

  AnyCollection msg;
  msg[0] = string("a");  //automatically sets msg to be an array
  msg[1]["foo"] = string("b"); //automatically sets the second element of msg to be a map
  msg[2].resize(4);   //automatically sets the third element to be an array with 4 empty elements
  msg[2][0] = 100;
  msg[2][1] = 101;
  msg[2][2] = 102;
  msg[2][3] = 103;
  msg["bar"] = string("c"); //automatically changes msg to be a map
  //msg["bar"].resize(3); //error here, can't resize a primitive value
  cout<<msg<<endl;    //prints out the JSON string encoding msg

[IMPORTANT NOTE: to assign a C-style string an element, cast it to a
std::string.]

To convert a message into your own data structures, you can either cast
to the desired type or use the as<T>() and asvector<T>() functions as follows:

  string elem1 = msg[0];     //elem1 is now "a"
  cout<<elem1<<" = a"<<endl;  //this prints "a = a"
  int itemp;
  if(msg[0].as<int>(itemp)) printf("Weird, msg[0] is converted to an integer\n");
  else printf("No, msg[0] is not an integer\n");  //this line gets printed
  cout<<msg[2].size()<<" = 4"<<endl;  //this prints 4 = 4
  vector<int> vtemp;
  if(!msg[2].asvector<int>(vtemp)) printf("Weird, msg[2] is not converted to a vector<int>\n");
  else printf("Yes, msg[2] is a vector<int> of size %d\n",vtemp.size()); // this line gets printed.
  //msg[2] is now an array [100,101,102,103]

The advantage of the asX() accessors is that they return false if the
conversion failed, which lets you perform error checking.

You can also access nested paths using the lookup(path,insert) function. If
insert is true, then a new element is added.  The path is a string that can
be given in either bracket notation:
  msg.lookup("[1][\"foo\"]"); //this a pointer to the element "b"
  msg.lookup("[1][\"bar\"]"); //this returns NULL
  *msg.lookup("[1][\"bar\"]",true) = string("d"); //this returns a pointer to a new AnyCollection
     //and assigns "d" to it
  msg.lookup("[\"bar\"]");  //this returns a pointer to the element "c"
or Javascript notation:
  msg.lookup("[1].foo");
  msg.lookup("[1].bar");
  *msg.lookup("[1].bar",true) = string("d");
  msg.lookup(".bar");  //note the leading '.'.  It is not strictly necessary and "bar" would work too.

You can iterate over arrays and maps as follows:
  vector<AnyKeyable> keys;
  vector<SmartPointer<AnyCollection> > elements;
  msg.enumerate_keys(keys);
  msg.enumerate_values(values);

See KrisLibrary/utils/AnyCollection for more documentation.

=== Programming Services ===

To build your own service, you should subclass SSPP::Service (Service.h) 
and override the OnMessage() method to read messages.  If you want to
send messages without being contacted by the other endpoint first, you
should overload the Process() method and call SendMessage() as needed. 
If you need to perform startup and/or shutdown tasks, overload OnStart()
and/or OnStop().

To run a service automatically, call RunForever, RunUntil, RunWhile, or
RunUntilKeypress (Service.h). You may also run multiple services
simultaneously using the versions that take a vector<Service*>.

See echo_service.cpp, log_service.cpp, heartbeat_service.cpp, and
send_service.cpp for examples.

=== Topic Services ===

To build your own service that connects to a topic server, subclass
SSPP::TopicServiceBase (Topic.h).



==========
 Python 
==========

=== Installing ===

You can simply copy the Python/sspp directory wherever you need it, or
install it to your python distribution by entering

  python setup.py install

or on Linux systems,

  sudo python setup.py install

in the ./Python directory.


=== Structure encoding and decoding ===

In Python, you create your structure as a possibly nested dictionary, array,
or primitive value (bool, int, float, or string).  When you call
JSONService.sendMessage it uses the json package to convert the object to
a string.  It does the reverse to decode the string to the structure passed to 
onMessage.


=== Programming Services ===

You will subclass sspp.Service and override the onMessage() and possibly the
onUpdate() methods.

See echo_service.py, log_service.py, and send_service.py for examples.

