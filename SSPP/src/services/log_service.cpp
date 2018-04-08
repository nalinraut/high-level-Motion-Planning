#include <sspp/Service.h>
#include <KrisLibrary/myfile.h>
using namespace std;

///This logs messages to a disk or plays back messages from disk
class LogService : public SSPP::Service
{
public:
	string filename;
	File f;
	bool replay;

	LogService() : replay(false) {}
	virtual const char* Name() const { if(replay) return "Log-Playback"; else return "Log"; }
	virtual const char* Description() const { if(replay) return "Plays back a log file"; else return "Logs messages to disk"; }
	virtual bool OnStart() {
		if(replay) {
			if(!f.Open(filename.c_str(),FILEREAD)) return false;
		}
		else {
			if(!f.Open(filename.c_str(),FILEWRITE)) return false;
		}
		return true;
	}
	virtual void OnClose() {
		f.Close();
	}
	virtual bool OnMessage(AnyCollection& message) {
		if(replay) return false;
		if(f.Position() < 0) return false;
		stringstream ss;
		ss<<message;
		return f.WriteString(ss.str().c_str());
	}
	virtual int Process() {
		int res = SSPP::Service::Process();
		if(replay) {
			char buf[0xffff];
			if(!f.ReadString(buf,0xffff)) {
				printf("Hit end of log file\n");
				return -1; //end of file
			}
			AnyCollection msg;
			stringstream ss(buf);
			ss >> msg;
			if(!ss) { 
				printf("Log file didn't contain a valid JSON structure\n");
			}
			if(!SendMessage(msg)) return -1;
		}
		return res;
	}
	virtual int ParseCommandLineOption(int argc,const char** argv,int index)
	{
		if(0==strcmp(argv[index],"--replay")) {
			replay = true;
			return index+1;
		}
		else if(0==strcmp(argv[index],"-f") || 0==strcmp(argv[index],"--file")) {
			filename = argv[index+1];
			return index+2;
		}
		else return SSPP::Service::ParseCommandLineOption(argc,argv,index);
	}
	virtual void PrintCommandLineOptions() const
	{
		SSPP::Service::PrintCommandLineOptions();
		printf(" --replay: replays a previously saved log file\n");
		printf(" -f,--file fn: sets the log file\n");
	}
};

int main(int argc,const char* argv[])
{
	LogService service;
	int i = service.ParseCommandLineOptions(argc,argv);
	if(i < 0) return 1;
	if(i == argc) {
		if(service.filename.empty()) {
			printf("USAGE: LogService [options] logfile\n");
			return -1;
		}
	}
	else if(i+1 <  argc) {
		printf("Warning, extra command line arguments...\n");
		service.PrintCommandLineOptions();
	}
	else
		service.filename = argv[i];

	if(!service.Connected()) {
		if(!service.OpenClient()) {
			fprintf(stderr,"Error opening client socket\n");
			return 1;
		}
	}
	SSPP::RunForever(service);
	return 0;
}
