#include <sspp/Send.h>

int main(int argc,const char* argv[])
{
	SSPP::SendService service;
	int i = service.ParseCommandLineOptions(argc,argv);
	if(i < 0) return 1;
	while(i != argc) {
		service.AddMessage(argv[i]);
		i+=1;
	}
	if(!service.Connected()) {
		if(!service.OpenClient()) {
			fprintf(stderr,"Error opening client socket\n");
			return 1;
		}
	}
	SSPP::RunForever(service);
	return 0;
}
