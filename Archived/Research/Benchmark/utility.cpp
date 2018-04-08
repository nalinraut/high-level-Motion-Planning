#include "utility.h"
using namespace std;

bool createDir(char* dir)
{
	struct stat folder_exist;
	if(stat(dir, &folder_exist) == 0 && S_ISDIR(folder_exist.st_mode)){
//				cout<<dir<<" folder exist!"<<endl;
		return true;
	}else{
		int res = mkdir(dir, S_IRWXU);
		if(res != 0 && errno != EEXIST){
			cout<<"failed mkdir "<<dir<<endl;
			cout<<"errno:"<<errno;
			return false;
		}
	}
	return true;
}

double determinantTriangleMatrix(const Math::Matrix &matrix)
{
    assert(matrix.m == matrix.n);
    double det = 1;
    for(int i = 0; i < matrix.m; i++)
    {
        det *= matrix(i,i);
    }
    det = det*det;
    return det;
}

void state_to_config(const ob::ScopedState<>& state, Config& config) {
	vector<double> values = state.reals();
	config = values;
}



//#include <ompl/base/spaces/RealVectorStateSpace.h>
//
//void tgfPlannerData::print(std::ostream &out) const
//{
//	for (unsigned int i = 0 ; i < states.size() ; ++i)
//	{
//		out << i+1 << " "<<2<<" ";
//		const ob::RealVectorStateSpace::StateType *tmp = states[i]->as<ob::RealVectorStateSpace::StateType>();
//		double xCoord=(*tmp)[0];
//		double yCoord=(*tmp)[1];
//		out << xCoord <<" "<<yCoord<< std::endl;
//	}
//	out<<"#"<<std::endl;
//	for (unsigned int i = 0 ; i < edges.size() ; ++i)
//	{
//		if (edges[i].empty())
//			continue;
//		for (unsigned int j = 0 ; j < edges[i].size() ; ++j)
//		{
//			out << i+1 << " ";
//			out << edges[i][j]+1 <<" "<<std::endl;
//		}
//	}
//}
//
//tgfPlannerData & tgfPlannerData::operator = (const tgfPlannerData & other)
//{
//	this->clear();
//	this->si=other.si;
//	this->states=other.states;
//	this->tags=other.tags;
//	this->stateIndex=other.stateIndex;
//	this->edges=other.edges;
//	this->properties=other.properties;
//	return *this;
//}

