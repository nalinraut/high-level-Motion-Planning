/*
 * SetupEnvironment.cpp
 *
 *  Created on: Jan 15, 2014
 *      Author: iuiml
 */

#include "SetupEnvironment.h"
#include "utility.h"
#include "PolyhedronStateChecker.h"
#include "LinkageStateChecker.h"

using namespace std;
using namespace Math;

double getBestPathCost(ScenarioIndex scenario, int nD, double width, double thin, double valuePerturb){
	double cost = 0;
	if(scenario == NarrowPassage_2_homotopy){
		cost = getBestPathCost_2_homotopy(nD, width, thin, valuePerturb);
	}else if(scenario == NarrowPassage_1_homotopy){
		cost = getBestPathCost_1_homotopy(nD, width, thin, valuePerturb);
	}else if(scenario == PlanaryLinkage){
		cost = getBestPathCost_Linkage(nD);
	}else if(scenario == NarrowKink_1_homotopy){
		cost =  getBestPathCost_1_Kink(nD, width, thin, valuePerturb);
	}
	return cost;
}

double getBestPathCost_Linkage(int nD){
	double cost = 0;
	for(int i = 0; i < nD; i++){
		double distance;
		if(nD == 5)
			distance = Math::Pi*(joint_goal_5D[i] - joint_start_5D[i])/180.0;
		if(nD == 6)
			distance = Math::Pi*(joint_goal_6D[i] - joint_start_6D[i])/180.0;
		if(nD == 7)
			distance = Math::Pi*(joint_goal_7D[i] - joint_start_7D[i])/180.0;
		if(nD == 8)
			distance = Math::Pi*(joint_goal_8D[i] - joint_start_8D[i])/180.0;
		if(distance < 0)
			distance = -distance;
		cost += distance;
	}
	return cost;
}

double getBestPathCost_1_Kink(int nD, double width, double thin, double valuePerturb){
    double onesecond = onesecond_global;
    int n_corners = num_config_best_path_kink/2;
    double low, high;
    getScenarioBounds(NarrowKink_1_homotopy, low, high);
    double corner_height = (high - low)/(double)n_corners;

    vector<Vector> bestpath;
    bestpath.resize(num_config_best_path_kink);

    assert(num_config_best_path_kink%4 == 0);
    int index = 0;
    int height_index = 0;
    while(index < num_config_best_path_kink){
    	Vector state1(nD, onesecond);
    	state1[0] = onesecond + thin/2.0 - width/2.0;
    	state1[nD-1] = height_index*corner_height + width/2.0;
    	bestpath[index] = state1;

    	height_index++;
    	Vector state2(nD, onesecond);
    	state2[0] = onesecond + thin/2.0 - width/2.0;
    	state2[nD-1] = height_index*corner_height - width/2.0;
    	bestpath[index+1] = state2;

    	Vector state3(nD, onesecond);
    	state3[0] = onesecond - thin/2.0 + width/2.0;
    	state3[nD-1] = height_index*corner_height + width/2.0;
    	bestpath[index+2] = state3;

    	height_index++;
    	Vector state4(nD, onesecond);
    	state4[0] = onesecond - thin/2.0 + width/2.0;
    	state4[nD-1] = height_index*corner_height - width/2.0;
    	bestpath[index+3] = state4;

    	index = index + 4;
    }

    Vector start(nD,onesecond),goal(nD,onesecond);
    start[0] = onesecond + thin/2.0;
    start[nD-1] = 0+0.0001;
    goal[0] = onesecond - thin/2.0;
    goal[nD-1] = 1-0.0001;
    bestpath[0] = start;
    bestpath[num_config_best_path_kink-1] = goal;

    double length = 0;
    for(int i= 0; i < num_config_best_path_kink-1; i++)
    {
        length = length + normalize(nD,bestpath[i],bestpath[i+1]);
    }

//    for(int i = 0; i < bestpath.size(); i++){
//    	cout<<i<<":"<<bestpath[i]<<endl;
//    }
//    cout<<"length:"<<length<<endl;getchar();
    return length;
}

double getBestPathCost_2_homotopy(int nD, double width, double thin, double valuePerturb)
{
    double onefourth_local = onefourth_global;
    double onesecond = onesecond_global;

    onefourth_local += valuePerturb;

    vector<Vector> bestpath;
    bestpath.resize(4);
    Vector state(nD);
    state.set(onesecond_global);
    state[0] = 0;
    state[nD-1] = 0;
    bestpath[0] = state;

    state.set(onesecond_global);
    state[0] = onefourth_local - width/2;
    state[nD-1] = onesecond_global - thin/2;
    bestpath[1] = state;

    state.set(onesecond_global);
    state[0] = onefourth_local - width/2;
    state[nD-1] = onesecond_global + thin/2;
    bestpath[2] = state;

    state.set(onesecond_global);
    state[0] = 0;
    state[nD-1] = 1;
    bestpath[3] = state;
    double length = 0;
    for(int i= 0; i < 3; i++)
    {
        length = length + normalize(nD,bestpath[i],bestpath[i+1]);
    }
    return length;
}

double getBestPathCost_1_homotopy(int nD, double width, double thin, double valuePerturb)
{
    double onesecond_local = onesecond_global;
    double onesecond = onesecond_global;

    onesecond_local += valuePerturb;

    vector<Vector> bestpath;
    bestpath.resize(4);
    Vector state(nD);
    state.set(onesecond_global);
    state[0] = 0;
    state[nD-1] = 0;
    bestpath[0] = state;

    state.set(onesecond_global);
    state[0] = onesecond_local - width/2;
    state[nD-1] = onesecond_global - thin/2;
    bestpath[1] = state;

    state.set(onesecond_global);
    state[0] = onesecond_local - width/2;
    state[nD-1] = onesecond_global + thin/2;
    bestpath[2] = state;

    state.set(onesecond_global);
    state[0] = 0;
    state[nD-1] = 1;
    bestpath[3] = state;
    double length = 0;
    for(int i= 0; i < 3; i++)
    {
        length = length + normalize(nD,bestpath[i],bestpath[i+1]);
    }
    return length;
}

bool readFile(const char* filename, int &nD, double &width, double &thin)
{
    ifstream file;
    file.open(filename, ios::in);
    if (file.is_open()) {
        while(!file.eof())
        {
            string line;
            getline(file, line);
            stringstream ss(stringstream::in | stringstream::out);
            if(!line.empty()&&line.at(0) == 'D')
            {
                getline(file,line);
                ss << line;
                ss >> nD;
            }else if(!line.empty()&&line.at(0) == 'T')
            {
                getline(file,line);
                ss << line;
                ss >> thin;
            }else if(!line.empty() && line.at(0) == 'W')
            {
                getline(file,line);
                ss << line;
                ss >> width;
            }
        }
    }else{
        cout<<"Cannot open the file for polyhedrons."<<endl;
        return false;
    }
    file.close();
    return true;
}

void setupStartandGoal(ScenarioIndex scenario, const int &nD, const double &thin, ob::ScopedState<> &start, ob::ScopedState<> &goal)
{
	if(scenario == NarrowPassage_2_homotopy || scenario == NarrowPassage_1_homotopy){
		double onefourth = onefourth_global;
		double onesecond = onesecond_global;
	//    if(perturbflag){
	//        onefourth = onefourthP;
	//        onesecond = onesecondP;
	//    }
		start[0] = 0;
		start[nD-1] = 0;
		goal[0] = 0;
		goal[nD-1] = 1;
		for(int i = 1; i < nD-1; i++)
		{
			start[i] = onesecond;
			goal[i] = onesecond;
		}
	}else if(scenario == NarrowKink_1_homotopy){
		double onefourth = onefourth_global;
		double onesecond = onesecond_global;

		for(int i = 0; i < nD; i++){
			start[i] = onesecond;
			goal[i] = onesecond;
		}
		start[0] = onesecond + thin/2.0;
		start[nD-1] = 0+0.0001;
		goal[0] = onesecond-thin/2.0;
		goal[nD-1] = 1-0.0001;
	}else{
		for(int i = 0; i < nD; i++){
			if(nD == 5){
				start[i] = joint_start_5D[i]*Math::Pi/180.0;
				goal[i] = joint_goal_5D[i]*Math::Pi/180.0;
			}
			if(nD == 6){
				start[i] = joint_start_6D[i]*Math::Pi/180.0;
				goal[i] = joint_goal_6D[i]*Math::Pi/180.0;
			}
			if(nD == 7){
				start[i] = joint_start_7D[i]*Math::Pi/180.0;
				goal[i] = joint_goal_7D[i]*Math::Pi/180.0;
			}
			if(nD == 8){
				start[i] = joint_start_8D[i]*Math::Pi/180.0;
				goal[i] = joint_goal_8D[i]*Math::Pi/180.0;
			}
		}
	}
}

void getScenarioBounds(ScenarioIndex scenario, double &low, double &high){
	if(scenario == PlanaryLinkage){
		low = -Math::Pi;
		high = Math::Pi;
	}else{
		low = 0;
		high = 1;
	}
}

ob::StateValidityChecker* setupNarrowPassage(ScenarioIndex scenario, ob::SpaceInformationPtr &si, const int &nD, const double &width, const double &thin, double valuePerturb){
	ob::StateValidityChecker* myChecker;
	if(scenario == NarrowPassage_2_homotopy){
		vector<Polyhedron> list = setupNarrowPassage_2_homotopy(nD, width, thin, valuePerturb);
		myChecker = new PolyhedronStateChecker(si);
		((PolyhedronStateChecker*)myChecker)->addPolyObt(list);
	}else if(scenario == NarrowPassage_1_homotopy){
		vector<Polyhedron> list = setupNarrowPassage_1_homotopy(nD, width, thin, valuePerturb);
		myChecker = new PolyhedronStateChecker(si);
		((PolyhedronStateChecker*)myChecker)->addPolyObt(list);
	}else if(scenario == PlanaryLinkage){
		myChecker = new LinkageStateChecker(si);
	}else if(scenario == NarrowKink_1_homotopy){
		vector<Polyhedron> list = setupNarrowPassage_1_Kink(nD, width, thin, valuePerturb);
		myChecker = new PolyhedronStateChecker(si);
		((PolyhedronStateChecker*)myChecker)->addPolyObt(list);
		((PolyhedronStateChecker*)myChecker)->setReverseValidity();
	}
    return myChecker;
}

vector<Polyhedron> setupNarrowPassage_1_Kink(const int &nD, const double &width, double thin, double valuePerturb){
	double high, low;
	getScenarioBounds(NarrowKink_1_homotopy, low,high);
    int n_corners = num_config_best_path_kink/2;
    double corner_height = (high - low)/(double)n_corners;
    assert(width <= corner_height);

    int mD = 2*nD;
	vector<Polyhedron> list;
    double onefourth = onefourth_global;
    double onesecond = onesecond_global;

    double *A = new double[mD*nD];
    double *b = new double[mD];
    double *s = new double[nD];
    double *e = new double[nD];

    for(int i = 0; i < mD*nD; i++)
    {
        A[i] = 0;
    }
    for(int i = 0; i < mD/2; i++)
    {
        A[i*nD+i] = 1;
        A[(i+mD/2)*nD+i] = -1;
    }

    int n_vertical_boxes_one_side = n_corners/2;
    for(int iter = 0; iter < n_vertical_boxes_one_side; iter++){
    	//vertical boxes on right side
    	    for(int i = 0; i < nD; i++)
    	    {
    	        s[i] = onesecond-width/2;
    	        e[i] = width;
    	    }
    	    s[0] = onesecond + thin/2.0 - width/2.0;
    	    if(iter == 0){
				s[nD-1] = 0;
				e[nD-1] = corner_height + width/2.0;
    	    }else{
				s[nD-1] = 2*iter*corner_height - width/2.0;
				e[nD-1] = corner_height + width;
    	    }
    	    for(int i = 0; i < nD; i++)
    	    {
    	        b[i] = s[i];
    	        b[nD+i] = (-1)*(s[i] + e[i]);
    	    }
    	    Polyhedron poly1(mD,nD,A,b);
    	    list.push_back(poly1);

    	    //vertical boxes on left side
			for(int i = 0; i < nD; i++)
			{
				s[i] = onesecond-width/2;
				e[i] = width;
			}
			s[0] = onesecond - thin/2.0 - width/2.0;
			if(iter == (n_vertical_boxes_one_side - 1)){
				s[nD-1] = (2*iter+1)*corner_height - width/2.0;;
				e[nD-1] = corner_height + width/2.0;
			}else{
				s[nD-1] = (2*iter+1)*corner_height - width/2.0;
				e[nD-1] = corner_height + width;
			}
			for(int i = 0; i < nD; i++)
			{
				b[i] = s[i];
				b[nD+i] = (-1)*(s[i] + e[i]);
			}
			Polyhedron poly2(mD,nD,A,b);
			list.push_back(poly2);
    }
//	horizontal obstacles
    int n_horizontal_boxes = n_corners - 1;
    for(int iter = 0; iter < n_horizontal_boxes; iter++){
   	    for(int i = 0; i < nD; i++)
		{
			s[i] = onesecond-width/2;
			e[i] = width;
		}
		s[0] = onesecond - thin/2.0 - width/2.0;
		e[0] = thin + width;
		s[nD-1] = (iter+1)*corner_height - width/2.0;
		for(int i = 0; i < nD; i++)
		{
			b[i] = s[i];
			b[nD+i] = (-1)*(s[i] + e[i]);
		}
		Polyhedron poly1(mD,nD,A,b);
		list.push_back(poly1);
    }
    delete []s;
    delete []e;
    delete []A;
    delete []b;
    return list;
}

vector<Polyhedron> setupNarrowPassage_2_homotopy(const int &nD, const double &width, const double &thin, double valuePerturb)
{
	int mD = 2*nD;
	vector<Polyhedron> list;
    double onefourth = onefourth_global;
    double onesecond = onesecond_global;

    double np_1stD_2ndobstacle_extent = 0.5;

    onefourth += valuePerturb;

    double *A = new double[mD*nD];
    double *b = new double[mD];
    for(int i = 0; i < mD*nD; i++)
    {
        A[i] = 0;
    }
    for(int i = 0; i < mD/2; i++)
    {
        A[i*nD+i] = 1;
        A[(i+mD/2)*nD+i] = -1;
    }
//first obstacle
    double *s = new double[nD];
    double *e = new double[nD];
    for(int i = 0; i < nD; i++)
    {
        s[i] = 0;
        e[i] = 1;
    }
    s[nD-1] = onesecond_global - thin/2;
    e[0] = onefourth - width/2;
    e[nD-1] = thin;
    for(int i = 0; i < nD; i++)
    {
        b[i] = s[i];
        b[nD+i] = (-1)*(s[i] + e[i]);
//        cout<<i<<":"<<b[i]<<endl<<b[nD+i]<<endl;
    }
    Polyhedron poly1(mD,nD,A,b);
    list.push_back(poly1);
//second obstacle
    for(int i = 0; i < nD; i++)
    {
        s[i] = 0;
        e[i] = 1;
    }
    s[0] = onefourth + width/2;
//    e[0] = onefourth - width/2;
    e[0] = np_1stD_2ndobstacle_extent - onefourth - width/2;
    s[nD-1] = onesecond_global - thin/2;
    e[nD-1] = thin;
    for(int i = 0; i < nD; i++)
    {
        b[i] = s[i];
        b[nD+i] = (-1)*(s[i] + e[i]);
    }
    Polyhedron poly2(mD,nD,A,b);
    list.push_back(poly2);
 //the rest obstacles
    for(int i = 1; i < (nD-1); i++)
    {
        for(int j = 0; j < nD; j++)
        {
            s[j] = 0;
            e[j] = 1;
        }
        s[0] = onefourth - width/2;
        e[0] = width;
        for(int j = 1; j < (i-1); j++)
        {
            s[j] = onesecond_global - width/2;
            e[j] = width;
        }
        s[i] = 0;
        e[i] = onesecond_global - width/2;
        s[nD-1] = onesecond_global - thin/2;
        e[nD-1] = thin;
        for(int j = 0; j < nD; j++)
        {
            b[j] = s[j];
            b[nD+j] = (-1)*(s[j] + e[j]);
        }
        Polyhedron poly_1(mD,nD,A,b);
        list.push_back(poly_1);
        /////////////////////////////
        for(int j = 0; j < nD; j++)
        {
            s[j] = 0;
            e[j] = 1;
        }
        s[0] = onefourth - width/2;
        e[0] = width;
        for(int j = 1; j < (i-1); j++)
        {
            s[j] = onesecond_global - width/2;
            e[j] = width;
        }
        s[i] = onesecond_global + width/2;
        e[i] = onesecond_global - width/2;
        s[nD-1] = onesecond_global - thin/2;
        e[nD-1] = thin;
        for(int j = 0; j < nD; j++)
        {
            b[j] = s[j];
            b[nD+j] = (-1)*(s[j] + e[j]);
        }
        Polyhedron poly_2(mD,nD,A,b);
        list.push_back(poly_2);
    }
    delete []s;
    delete []e;
    delete []A;
    delete []b;

    return list;
}

vector<Polyhedron> setupNarrowPassage_1_homotopy(const int &nD, const double &width, const double &thin, double valuePerturb)
{
	int mD = 2*nD;
    vector<Polyhedron> list;
    double onesecond_local = onesecond_global;
    double onesecond = onesecond_global;
    double np_1stD_2ndobstacle_extent = 1;

    onesecond_local += valuePerturb;

    double *A = new double[mD*nD];
    double *b = new double[mD];
    for(int i = 0; i < mD*nD; i++)
    {
        A[i] = 0;
    }
    for(int i = 0; i < mD/2; i++)
    {
        A[i*nD+i] = 1;
        A[(i+mD/2)*nD+i] = -1;
    }
//first obstacle
    double *s = new double[nD];
    double *e = new double[nD];
    for(int i = 0; i < nD; i++)
    {
        s[i] = 0;
        e[i] = 1;
    }
    s[nD-1] = onesecond_global - thin/2;
    e[0] = onesecond_local - width/2;
    e[nD-1] = thin;
    for(int i = 0; i < nD; i++)
    {
        b[i] = s[i];
        b[nD+i] = (-1)*(s[i] + e[i]);
//        cout<<i<<":"<<b[i]<<endl<<b[nD+i]<<endl;
    }
    Polyhedron poly1(mD,nD,A,b);
    list.push_back(poly1);
//second obstacle
    for(int i = 0; i < nD; i++)
    {
        s[i] = 0;
        e[i] = 1;
    }
    s[0] = onesecond_local + width/2;
//    e[0] = onesecond_local - width/2;
    e[0] = np_1stD_2ndobstacle_extent - onesecond_local - width/2;
    s[nD-1] = onesecond_global - thin/2;
    e[nD-1] = thin;
    for(int i = 0; i < nD; i++)
    {
        b[i] = s[i];
        b[nD+i] = (-1)*(s[i] + e[i]);
    }
    Polyhedron poly2(mD,nD,A,b);
    list.push_back(poly2);
 //the rest obstacles
    for(int i = 1; i < (nD-1); i++)
    {
        for(int j = 0; j < nD; j++)
        {
            s[j] = 0;
            e[j] = 1;
        }
        s[0] = onesecond_local - width/2;
        e[0] = width;
        for(int j = 1; j < (i-1); j++)
        {
            s[j] = onesecond_global - width/2;
            e[j] = width;
        }
        s[i] = 0;
        e[i] = onesecond_global - width/2;
        s[nD-1] = onesecond_global - thin/2;
        e[nD-1] = thin;
        for(int j = 0; j < nD; j++)
        {
            b[j] = s[j];
            b[nD+j] = (-1)*(s[j] + e[j]);
        }
        Polyhedron poly_1(mD,nD,A,b);
        list.push_back(poly_1);
        /////////////////////////////
        for(int j = 0; j < nD; j++)
        {
            s[j] = 0;
            e[j] = 1;
        }
        s[0] = onesecond_local - width/2;
        e[0] = width;
        for(int j = 1; j < (i-1); j++)
        {
            s[j] = onesecond_global - width/2;
            e[j] = width;
        }
        s[i] = onesecond_global + width/2;
        e[i] = onesecond_global - width/2;
        s[nD-1] = onesecond_global - thin/2;
        e[nD-1] = thin;
        for(int j = 0; j < nD; j++)
        {
            b[j] = s[j];
            b[nD+j] = (-1)*(s[j] + e[j]);
        }
        Polyhedron poly_2(mD,nD,A,b);
        list.push_back(poly_2);
    }
    delete []s;
    delete []e;
    delete []A;
    delete []b;

    return list;
}

vector<Polyhedron> readNarrowPassage(const char* filename, CEState &sp, CEState &gp, int &D)
{
    int mD,nD;
    double thin,width;

    if(!readFile(filename,nD,width,thin))
    {
        cout<<"cannot read the narrow passage fiel."<<endl;
        vector<Polyhedron> tmp;
        return tmp;
    }
    D = nD;
    mD = 2*nD;
//    cout<<mD<<endl<<nD<<endl<<width<<endl<<thin<<endl;
// setup start and goal ///////////////////////////////////////////////////
    sp.q.resize(nD);
    gp.q.resize(nD);
    sp.q[0] = 0;
    sp.q[nD-1] = 0;
    gp.q[0] = 0;
    gp.q[nD-1] = 1;
    for(int i = 1; i < nD-1; i++)
    {
        sp.q[i] = onesecond_global;
        gp.q[i] = onesecond_global;
    }
// setup polyhedrons ///////////////////////////////////////////////////
    vector<Polyhedron> list = setupNarrowPassage_2_homotopy(mD, nD, width, thin);

    return list;
}



