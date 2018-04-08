#include <cstdlib>
#include <math.h>
#include <fstream>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "FMM.h"
#include "utility.h"
#include "PolyhedronStateChecker.h"
namespace ob = ompl::base;
namespace og = ompl::geometric;

FMM::FMM(const ob::SpaceInformationPtr& si, ob::ScopedState<>& start, ob::ScopedState<>& goal) : ob::Planner(si, "FMM") {
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;

    this->initializeParameters();
    this->setStartandGoal(start, goal);
}

void FMM::initializeParameters() {
    nD = ob::Planner::si_->getStateDimension();
    ob::RealVectorBounds bounds = ob::Planner::si_->getStateSpace()->as<ob::RealVectorStateSpace > ()->getBounds();
    lowbound = bounds.low[0];
    highbound = bounds.high[0];
//    numGrid = 2;
//TODO,change back
    numGrid = 4;
    lastnumGrid = 0;    
    coefficient_nGrid = pow(2, 1 / (double)nD);
    grid_resolution = (highbound - lowbound) / (double)numGrid;
    cc_resolution = ob::Planner::si_->getStateValidityCheckingResolution() * ob::Planner::si_->getMaximumExtent();
    colcheckTime = 0;
    gridoffset = 0;
    thin_of_narrow_passage = 0;
}

void FMM::mapStatetoNearestGridPoint(const Vector& state, vector<int>& gridIndex) {
	vector<int> gridLCorner = this->stateToIndex(state);
	assert(gridLCorner.size() == this->nD);
	Vector stateLCorner, bestneighbor;
	stateLCorner = this->indexToState(gridLCorner);

	//map state to grid lowest corner
	double diff_threshold = 0.0001;
	double minDisttoGrid = (state - stateLCorner).norm();
	gridIndex = gridLCorner;
	bestneighbor = stateLCorner;
	if(minDisttoGrid > diff_threshold){
		vector<int> gridHCorner;gridHCorner.resize(gridLCorner.size());
		for(int i = 0; i < gridLCorner.size(); i++){
			gridHCorner[i] = gridLCorner[i]+1;
		}

		Vector stateHCorner = this->indexToState(gridHCorner);
		double dist = (state - stateHCorner).norm();
		if(minDisttoGrid > dist){
			minDisttoGrid = dist;
			gridIndex = gridHCorner;
			bestneighbor = stateHCorner;
		}
		//map state to grid highest corner if the distance is less than threshold
		if(minDisttoGrid < diff_threshold)
			return;
		for(int i = 0; i < gridLCorner.size(); i++){
			vector<int> gridOneCorner = gridLCorner;
			gridOneCorner[i] += 1;
			Vector stateOneCorner = this->indexToState(gridOneCorner);
			dist = (state - stateOneCorner).norm();
			if(minDisttoGrid > dist){
				minDisttoGrid = dist;
				gridIndex = gridOneCorner;
				bestneighbor = stateOneCorner;
			}

			gridOneCorner = gridHCorner;
			gridOneCorner[i] -= 1;
			stateOneCorner = this->indexToState(gridOneCorner);
			dist = (state - stateOneCorner).norm();
			if(minDisttoGrid > dist){
				minDisttoGrid = dist;
				gridIndex = gridOneCorner;
				bestneighbor = stateOneCorner;
			}
		}
	}
//	cout<<"gridIndex:"<<endl;
//	this->printIndex(gridIndex);
//	cout<<"numGrids:"<<numGrid<<endl;
//	cout<<"grid_resolution:"<<grid_resolution<<endl;
//	cout<<"state:"<<state<<endl;
//	cout<<"stateLCorner:"<<stateLCorner<<endl;
//	cout<<"bestneighbor:"<<bestneighbor<<endl;
//	this->printIndex(gridIndex);
//	cout<<"-----------------"<<endl;
}


void FMM::setStartandGoal(ob::ScopedState<>& s, ob::ScopedState<>& g) {
    this->realStart = s.reals();
    this->realGoal = g.reals();
}

vector<int> FMM::stateToIndex(const Vector& state) {
    vector<int> index;
    index.resize(nD);
    for (int i = 0; i < nD; i++) {
        index[i] = floor((state[i]-lowbound) / grid_resolution);
    }
    return index;
}

Vector FMM::indexToState(const vector<int>& index) {
    Vector state;
    state.resize(nD);
    for (int i = 0; i < nD; i++) {
        state[i] = lowbound + index[i] * grid_resolution;
    }
    return state;
}

Vector FMM::indexToState(const Vector& index) {
    Vector state;
    state.resize(nD);
    for (int i = 0; i < nD; i++) {
        state[i] = lowbound + index[i] * grid_resolution;
    }
    return state;
}

double FMM::getStateCost(const Vector& state) {
	double t1 = elapsedTime();
	double res = Inf;
    if (this->isStateValid(state))
    	res = 1;
    this->colcheckTime += elapsedTime() - t1;
//    cout<<"state:"<<state<<";res:"<<res<<endl;getchar();
    return res;
}

bool FMM::isStateinCollisionCells(const vector<int>& index){
	for(int i = 0; i < this->collision_points.size(); i++){
		vector<int> collisionLIndex = this->stateToIndex(collision_points[i]);
		bool incollision = true;
		for(int j = 0; j < index.size(); j++){
			if(index[j] != collisionLIndex[j] && index[j] != collisionLIndex[j]+1){
				incollision = false;
				break;
			}
		}
		if(incollision == true){
			return true;
		}
	}
	return false;
}

double FMM::getStateCost(const vector<int>& index) {
	if(isStateinCollisionCells(index) == true){
		return Inf;
	}
    Vector state = this->indexToState(index);
    return this->getStateCost(state);
}

bool FMM::checkStartandGoal() {
    if (!this->isStateValid(this->realStart))
        return false;
    if (!this->isStateValid(this->realGoal))
        return false;
    return true;
}

double FMM::getColCheckTime() {
    return this->colcheckTime;
}

void FMM::setup() {
    ob::Planner::setup();

    if (!checkStartandGoal()) {
        cout << "Start or Goal is not feasible." << endl;
        return;
    }

    // perhaps attempt some auto-configuration
    //  ompl::SelfConfig sc(si_, getName());
    //  sc.configure...
    //  sc.configurePlannerRange()
}

void FMM::getPlannerData(ob::PlannerData& data) const {
    // fill data with the states and edges that were created
    // in the exploration data structure
    // perhaps also fill control::PlannerData
    ob::StateSpacePtr sp = si_->getStateSpace();
    ob::State *state = (sp->as<ob::RealVectorStateSpace > ()->allocState());
    og::PathGeometric *pathGeometric = new og::PathGeometric(si_);
    for (int i = 0; i < solution.size(); i++) {
        for (int j = 0; j < nD; j++) {
            (state->as<ob::RealVectorStateSpace::StateType > ())->values[j] = (solution[i])[j];
        }
        pathGeometric->append(si_->cloneState(state));
    }
    pathGeometric->append(si_->cloneState(state));
    si_->freeState(state);

    ob::Planner::getPlannerData(data);

    for (unsigned int j = 0; j < solution.size() - 1; j++) {
        data.addEdge(pathGeometric->getState(j), pathGeometric->getState(j + 1));
    }
}

void FMM::clear() {
    ob::Planner::clear();
}

ob::PlannerStatus FMM::solve(const ob::PlannerTerminationCondition& ptc) {
    if (lastnumGrid == 0)
        lastnumGrid = numGrid;
    else {
//        numGrid = 2*lastnumGrid;
        numGrid = (int) ceil(lastnumGrid * coefficient_nGrid);
        if(numGrid % 2 == 1)
            numGrid = numGrid+1;
        lastnumGrid = numGrid;
    }
    grid_resolution = (highbound - lowbound) / (double)numGrid;
//    this->startIndex = this->stateToIndex(realStart);
//    this->goalIndex = this->stateToIndex(realGoal);
    this->mapStatetoNearestGridPoint(realStart, startIndex);
    this->mapStatetoNearestGridPoint(realGoal, goalIndex);
//    cout<<"startIndex:";
//    for(int i = 0; i < startIndex.size(); i++){
//    	cout<<startIndex[i]<<",";
//    }
//    cout<<endl;
//    cout<<"goalIndex:";
//    for(int i = 0; i < goalIndex.size(); i++){
//    	cout<<goalIndex[i]<<",";
//    }
//    cout<<endl;

    vector<int> size(nD);
    fill(size.begin(), size.end(), numGrid + 1);
    distances.resize(size);
    bool isSolved = solve();
    if (isSolved) {
        Vector goalV, startV;
        goalV.resize(goalIndex.size());
        startV.resize(startIndex.size());
        for (int k = 0; k < goalIndex.size(); k++) {
            goalV[k] = goalIndex[k];
            startV[k] = startIndex[k];
        }

        vector<Vector> solutionIndex = this->Descend(distances, goalV);
        cout<<"solutionIndex----------------:"<<solutionIndex.size()<<endl;
        for(int i = 0; i < solutionIndex.size(); i++){
        	cout<<solutionIndex[i]<<endl;
        }
        cout<<"--------------------"<<endl;
        Vector last = solutionIndex[solutionIndex.size() - 1];
        for (int k = 0; k < last.size(); k++) {
            if (startV[k] != last[k]) {
                solutionIndex.push_back(startV);
                break;
            }
        }
        //////////////////////
//        Vector lowbounds(nD,lowbound);
        solution.resize(solutionIndex.size());
        for (int k = 0; k < solutionIndex.size(); k++) {
        	solution[k] = this->indexToState(solutionIndex[k]);
//            solution[k] = lowbounds + solutionIndex[k] * this->grid_resolution;
        }
        ////////////////////add start and goal to solution path if they are not on grid points
        Vector startStateV = this->indexToState(startIndex);
        Vector goalStateV = this->indexToState(goalIndex);
        for (int k = 0; k < last.size(); k++) {
            if (startStateV[k] != realStart[k]) {
            	solution.push_back(realStart);
                break;
            }
        }
        for (int k = 0; k < last.size(); k++) {
            if (goalStateV[k] != realGoal[k]) {
            	solution.insert(solution.begin(),realGoal);
                break;
            }
        }

        if (this->isPathValid(solution)) {
            this->validflag = true;
        }else{
            this->validflag = false;
        }
//        cout<<"solution:"<<validflag<<endl;
//        for(int i = 0; i < solution.size(); i++){
//        	cout<<solution[i]<<endl;
//        }
//        getchar();
    }
//    if(this->validflag)
    if(isSolved)
        return ob::PlannerStatus::APPROXIMATE_SOLUTION;
    return ob::PlannerStatus::TIMEOUT;
}

bool FMM::solve() {
    Timer timer;
    initTime = 0; estimateTime = 0; propagateTime = 0; overheadTime = 0;
    int numVisited = 0, numSimplices = 0;
    assert(startIndex.size() == distances.dims.size());

    //initialize search status
    ArrayND<int> status;
    status.resize(distances.dims);
    status.set(0);

    ArrayND<Real> costs;
    costs.resize(distances.dims);
    costs.set(-1);
    //    distances.resize(costs.dims);
    distances.set(Inf);

    int sindex = distances.indexToOffset(startIndex);
    int gindex = -1;
    if (goalIndex.size() == startIndex.size()) gindex = distances.indexToOffset(goalIndex);

	//build queue
    FixedSizeHeap<Real> q(status.numValues());
    q.push(sindex, 0.0);
    status.values[sindex] = 1;

    initTime = timer.ElapsedTime();
    timer.Reset();

    //////////////////////////////////////
    costs.set(Inf);
    //////////////////////////////////////
    while (!q.empty()) {
        int nindex = q.top();
        q.pop();
        numVisited++;

        //set to visited
        status.values[nindex] = -1;

        //look at visited neighbors
        Real best = Inf;
        if (nindex == sindex)
            best = 0.0;

        //        Real c = costs.values[nindex];
        //        vector<int> node = costs.offsetToIndex(nindex); 
        vector<int> node = distances.offsetToIndex(nindex);
        Real c = 1;
//        if((nindex == sindex || nindex == gindex)&& costs.values[nindex] < 0){
//        	c = 1;
//        	costs.values[nindex] = c;
//        }else if (costs.values[nindex] < 0) {
//            c = this->getStateCost(node);
//            costs.values[nindex] = c;
//        } else {
//            c = costs.values[nindex];
//        }

        if((nindex == sindex || nindex == gindex)){
        	c = 1;
        	costs.values[nindex] = c;
        }else {
            c = this->getStateCost(node);
            costs.values[nindex] = c;
        }

        overheadTime += timer.ElapsedTime();
        timer.Reset();
        if (nindex != sindex) {
            SimplexEnumerator s(node, status, -1);
            vector<int> simplex;
            Vector d;
            do {
                numSimplices++;
                s.getOffsets(simplex);
                if (simplex.size() == 1)
                    best = Min(best, distances.values[simplex[0]] + c);
                else {
                    d.resize(simplex.size());
                    for (size_t i = 0; i < simplex.size(); i++)
                        d[i] = distances.values[simplex[i]];
                    best = Min(best, best_diag_distanceN(d) * c);
                }
            } while (s.next());
            estimateTime += timer.ElapsedTime();
            timer.Reset();
        }

        distances.values[nindex] = best;
        cout<<"numgrids:"<<this->numGrid<<endl;
    	cout<<"cost----------------\n";
    	for(int i = costs.dims[0]-1; i >=0; i--){
        	for(int j = 0; j < costs.dims[1]; j++){
        		vector<int> tmpindex;tmpindex.push_back(j);tmpindex.push_back(i);
        		int tmp = costs.indexToOffset(tmpindex);
       		cout<<costs.values[tmp]<<"\t";
        	}
        	cout<<endl;
    	}
    	cout<<"dist----------------\n";
    	for(int i = distances.dims[0]-1; i >=0; i--){
        	for(int j = 0; j < distances.dims[1]; j++){
        		vector<int> tmpindex;tmpindex.push_back(j);tmpindex.push_back(i);
        		int tmp = distances.indexToOffset(tmpindex);
        		cout<<distances.values[tmp]<<"\t";
        	}
        	cout<<endl;
    	}
    	if(numGrid == 16)
    		getchar();

        //check if goal is reached
        if (nindex == gindex) {
        	cout<<"cost----------------\n";
        	for(int i = costs.dims[0]-1; i >=0; i--){
            	for(int j = 0; j < costs.dims[1]; j++){
            		vector<int> tmpindex;tmpindex.push_back(j);tmpindex.push_back(i);
            		int tmp = costs.indexToOffset(tmpindex);
           		cout<<costs.values[tmp]<<"\t";
            	}
            	cout<<endl;
        	}
        	cout<<"dist----------------\n";
        	for(int i = distances.dims[0]-1; i >=0; i--){
            	for(int j = 0; j < distances.dims[1]; j++){
            		vector<int> tmpindex;tmpindex.push_back(j);tmpindex.push_back(i);
            		int tmp = distances.indexToOffset(tmpindex);
            		cout<<distances.values[tmp]<<"\t";
            	}
            	cout<<endl;
        	}
        	if(IsInf(this->distances.values[nindex]))
        		return false;
        	else
        		return true;
        }

        //otherwise, propagate to children
        vector<int> adjacent;
        Adjacent(node, nindex, distances, adjacent);
        for (size_t i = 0; i < adjacent.size(); i++) {
            int next = adjacent[i];
            //            Real ncost = best + costs.values[next];
            //            vector<int> nextIndex = distances.offsetToIndex(next);
            //            Real ncost = best + this->getStateCost(nextIndex);
            if (status.values[next] == 0) {
                vector<int> nextIndex = distances.offsetToIndex(next);
                costs.values[next] = this->getStateCost(nextIndex);

                Real ncost = best + costs.values[next];
                q.push(next, -ncost);
                distances.values[next] = ncost;
                status.values[next] = 1;
            } else if (status.values[next] == 1) {
                //do we have a better distance than the existing?
                Real ncost = best + costs.values[next];
                if (ncost < distances.values[next]) {
                    if (!q.find(next)) {
                        printf("Warning, want to adjust %d but couldn't find in queue\n", next);
                        q.push(next, -ncost);
                    } else {
                        q.adjust(next, -ncost);
                    }
                }
            } else {
                Real ncost = best + costs.values[next];
                if (ncost < distances.values[next]) {
                    //printf("Hmm... better path to complete node!\n");
                    //printf("Difference: %g vs %g\n",ncost,distances.values[next]);
                    distances.values[next] = ncost;
                }
            }
        }

        propagateTime += timer.ElapsedTime();
        timer.Reset();
    }
    //couldn't find goal
    return false;
}

void FMM::printIndex(const vector<int>& index){
	for(int i = 0; i < index.size(); i++){
		cout<<index[i]<<";";
	}
	cout<<endl;
}

void FMM::printSolution(std::ostream& out) {
	out<<"FMM numGrid "<<numGrid<<endl;
	for(int i = 0; i < solution.size(); i++){
		for(int j = 0; j < solution[i].size(); j++)
			out<<solution[i][j]<<" ";
		out<<endl;
	}
	out<<endl;
}

void FMM::setLowerBoundOffset(double offset) {
	this->lowbound += offset;
//	this->highbound += offset;
    grid_resolution = (highbound - lowbound) / (double)numGrid;
    gridoffset = offset;
}

void FMM::Adjacent(const vector<int>& index, int offset, const ArrayND<Real>& grid, vector<int>& noffsets) {
    noffsets.resize(0);
    noffsets.reserve(index.size()*2);
    vector<int> temp = index;
    for (size_t i = 0; i < index.size(); i++) {
        temp[i] += 1;
        if (temp[i] < grid.dims[i]) {
            noffsets.push_back(offset + grid.strides[i]);
        }
        temp[i] -= 2;
        if (temp[i] >= 0) {
            noffsets.push_back(offset - grid.strides[i]);
        }
        temp[i] += 1;
    }
}

Real FMM::best_diag_distanceN(const Vector& d) {
    /* Lagrange equation: d'(u,v,w) + lambda (1,1,1) = 0
      d' = (d1,d2,d3) + (u,v,w)/||u,v,w||
      (d1+lambda,d2+lambda,d3+lambda)||u,v,w|| = (-u,-v,-w)
      (d1+lambda)^2 ||u,v,w||^2 = u^2
      (d2+lambda)^2 ||u,v,w||^2 = v^2
      (d3+lambda)^2 ||u,v,w||^2 = w^2
      [(d1+lambda)^2 + (d2+lambda)^2 + (d3+lambda)^2]||u,v,w||^2 = ||u,v,w||^2
      (d1+lambda)^2 + (d2+lambda)^2 + (d3+lambda)^2 = 1
      3 lambda^2 + (2d1+2d2+2d3) lambda + (d1^2+d2^2+d3^2-1) = 0
      lambda = -(d1+d2+d3)/3 +/- 1/3 sqrt((d1+d2+d3)^2 - 3 (d1^2+d2^2+d3^2-1))
     */

    Real dsum = Sum(d);
    int n = d.n;
    Real dsumsq = Sqr(dsum);
    Real dsqsum = d.normSquared();
    Real det = dsumsq - n * (dsqsum - 1.0);
    if (det < 0.0) {
        //fprintf(stderr,"Negative determinant: %g\n",det);
        //cerr<<"D: "<<d<<endl;
        return d.minElement() + 1.0;
    }
    Real sqdet = Sqrt(det);
    Real lambda1 = (-dsum + sqdet) / n;
    Real lambda2 = (-dsum - sqdet) / n;
    Vector u1 = d, u2 = d;
    for (int i = 0; i < d.n; i++)
        u1[i] += lambda1;
    for (int i = 0; i < d.n; i++)
        u2[i] += lambda2;

    u1 *= 1.0 / (dsum + lambda1 * n);
    u2 *= 1.0 / (dsum + lambda2 * n);
    Real val1 = Inf, val2 = Inf;
    if (u1.minElement() >= 0.0 && u1.maxElement() <= 1.0)
        val1 = u1.dot(d) + u1.norm();
    if (u2.minElement() >= 0.0 && u2.maxElement() <= 1.0)
        val2 = u2.dot(d) + u2.norm();
    return Min(val1, val2);
}

/** Multilinear interpolation of an ND field.
* Sensitive to Inf's in the field -- will ignore them
*/
Real FMM::EvalMultilinear(const ArrayND<Real>& field,const Vector& point)
{
  vector<int> low(point.size());
  Vector u(point.n);
  for(size_t i=0;i<low.size();i++) {
    Real fp = Floor(point[i]);
    low[i] = (int)fp;
    u[i] = point[i] - fp;

    if(low[i] < 0) {
      low[i] = 0;
      u[i] = 0;
    }
    else if(low[i] >= field.dims[i]-1) {
      low[i] = field.dims[i]-2;
      u[i] = 1;
    }
  }
  vector<int> high(point.size());
  for(size_t i=0;i<low.size();i++)
    high[i] = low[i]+1;

  vector<int> vertex(low);
  Real s = 0.0;
  Real sumcoeff = 0.0;
  do {
    Real f=field[vertex];
    Real coeff = 1.0;
    for(size_t i=0;i<u.size();i++) {
      Real axisCoeff = (vertex[i]==low[i]? 1.0-u[i] : u[i]);
      coeff *= axisCoeff;
    }
    if(IsInf(f)) {
      if(coeff != 0) return Inf;
      else continue;
    }
    s += coeff*f;
    sumcoeff += coeff;
  } while(!IncrementIndex(vertex,low,high));
  if (sumcoeff == 0.0) return Inf;
  return s/sumcoeff;
}

#define INF_POS 1
#define INF_NEG 2
Vector FMM::FiniteDifference(const ArrayND<Real>& field,const Vector& x,vector<int>& infDirs)
{
  infDirs.resize(x.n);
  fill(infDirs.begin(),infDirs.end(),0);
  Real h = 0.25;
  Vector grad(x.size());
  Vector tmp = x;
  for(size_t i=0;i<x.size();i++) {
    tmp[i] = Min(x[i]+h,Real(field.dims[i])-1);
    Real f2 = EvalMultilinear(field,tmp);
    Real d = tmp[i] - x[i];
    if(IsInf(f2)) {
      infDirs[i] |= INF_POS;
      f2 = EvalMultilinear(field,x);
      d = 0.0;
    }
    tmp[i] = Max(x[i]-h,0.0);
    Real f1 = EvalMultilinear(field,tmp);
    if(IsInf(f1)) {
      infDirs[i] |= INF_NEG;
      f1 = EvalMultilinear(field,x);
    }
    else
      d += x[i] - tmp[i];
    tmp[i] = x[i];
    if(d == 0.0) grad[i] = 0.0;
    else grad[i] = (f2-f1)/d;
  }
  return grad;
}

/** Gradient descent of an ND field */
vector<Vector> FMM::Descend(const ArrayND<Real>& field,const Vector& start)
{
  Vector pt = start;
  vector<Vector> path;
  while(1) {
    path.push_back(pt);
    vector<int> infDirs;
    Vector grad = FiniteDifference(field,pt,infDirs);
    Real t=Inf;
    //do a line search to find the step size that steps out of the cell
    int bdry = 0;
    for(size_t i=0;i<pt.size();i++) {
      if(pt[i] <= 0 && grad[i] > 0) grad[i]=0.0;
      if(pt[i] >= field.dims[i]-1 && grad[i] < 0) grad[i]=0.0;
      if(infDirs[i] & INF_NEG && grad[i] > 0) grad[i]=0;
      if(infDirs[i] & INF_POS && grad[i] < 0) grad[i]=0;
      if(grad[i] == 0) continue;
      Real cell = Floor(pt[i]);
      if(pt[i] == cell) { //on this boundary, allow backwards movement
                if(pt[i]-t*grad[i] < cell-1) {
                  t = (pt[i]-cell+1)/grad[i];
                  bdry = i;
                }
      }
      else {
                if(pt[i]-t*grad[i] < cell) {
                  t = (pt[i]-cell)/grad[i];
                  bdry = i;
                }
      }
      if(pt[i]-t*grad[i] > cell+1) {
                t = (pt[i]-cell-1)/grad[i];
                bdry = i;
      }
    }
    if(IsInf(t)) {
//      cout<<"Terminated, gradient is zero"<<endl;
      return path;
    }
    Vector next = pt-grad*t;
    //round to nearest grid cell
    next[bdry] = Real(int(next[bdry]));
    /*
    for(size_t i=0;i<next.size();i++) {
      if(next[i] < 0.0) next[i] = 0.0;
      if(next[i] >= field.dims[i]) next[i] = field.dims[i]-1;
    }
    */
    if (EvalMultilinear(field,next) >= EvalMultilinear(field,pt)) {
//      cout<<"Terminated, next point "<<next<<" has cost "<<EvalMultilinear(field,next)<<", increase from "<<EvalMultilinear(field,pt)<<endl;
//      cout<<"inf dirs: ";
//      for(size_t i=0;i<infDirs.size();i++)
//                cout<<infDirs[i]<<" ";
//      cout<<endl;
      return path;
    }
    pt = next;
  }
  return path;
}

/////////////////////////////////////////////////////////////////////////////////////original
//vector<Vector> FMM::Descend(const ArrayND<Real>& field, const Vector& start) {
//    Vector grad = FiniteDifference(field, start);
//    Real maxval = grad.maxAbsElement();
//    vector<Vector> path(1, start);
//    if (maxval == 0)
//        return path;
//    //move by one pixel
//    grad *= 1.0 / maxval;
//    Vector next = start - grad;
//    for (size_t i = 0; i < next.size(); i++) {
//        if (next[i] < 0.0) next[i] = 0.0;
//        if (next[i] >= field.dims[i]) next[i] = field.dims[i] - 1;
//    }
//    if (EvalMultilinear(field, next) >= EvalMultilinear(field, start))
//        return path;
//    vector<Vector> suffix = Descend(field, next);
//    path.insert(path.end(), suffix.begin(), suffix.end());
//    return path;
//}
/////////////////////////////////////////////////////////////////////////////////////original

/////////////////////////////////////////////////////////////////////////////////////original
//Vector FMM::FiniteDifference(const ArrayND<Real>& field, const Vector& x) {
////        cout << "**********IN finiteDifference:x:" << x << endl;
//    Vector grad(x.size());
//    Vector tmp = x;
//    for (size_t i = 0; i < x.size(); i++) {
//        tmp[i] = Min(x[i] + 1.0, Real(field.dims[i]) - 1);
//
//        Real f2 = EvalMultilinear(field, tmp);
//        Real d = tmp[i] - x[i];
//        if (IsInf(f2)) {
//            f2 = EvalMultilinear(field, x);
//            d = 0.0;
//        }
//
//        tmp[i] = Max(x[i] - 1.0, 0.0);
//
//        Real f1 = EvalMultilinear(field, tmp);
//        if (IsInf(f1)) {
//            f1 = EvalMultilinear(field, x);
//        } else
//            d += x[i] - tmp[i];
//
//        tmp[i] = x[i];
//        if (d == 0.0) grad[i] = 0.0;
//        else grad[i] = (f2 - f1) / d;
//    }
//    return grad;
//}
//
//Real FMM::EvalMultilinear(const ArrayND<Real>& field, const Vector& point) {
////    cout << "**********IN EvalMultilinear:point:" << point << endl;
//   vector<int> low(point.size());
//    Vector u(point.n);
//    for (size_t i = 0; i < low.size(); i++) {
////    	cout<<"point[i]:"<<point[i]<<endl;
//        Real fp = Floor(point[i]);
//        low[i] = (int) fp;
//        u[i] = point[i] - fp;
//        /////////////////////////////////////////////////////////////
//        if (low[i] < 0) {
//            low[i] = 0;
//            u[i] = 0;
//        }
//        else if (low[i] >= field.dims[i] - 1) {
//            low[i] = field.dims[i] - 2;
//            u[i] = 1;
//        }
//        /////////////////////////////////////////////////////////////
//
//    }
//    vector<int> high(point.size());
//    for (size_t i = 0; i < low.size(); i++){
//        high[i] = low[i] + 1;
//        /////////////////////////////////////////////////////////////
//        if(high[i] > field.dims[i]-1)
//        	high[i] = field.dims[i]-1;
//        /////////////////////////////////////////////////////////////
//    }
//    vector<int> vertex(low);
//    Real s = 0.0;
//    Real sumcoeff = 0.0;
//    do {
//    	Real f = field[vertex];
//    	if (IsInf(f)) continue;
//
//    	Real coeff = 1.0;
//        for (size_t i = 0; i < u.size(); i++) {
//            coeff *= (vertex[i] == low[i] ? 1.0 - u[i] : u[i]);
//        }
//        s += coeff*f;
//        sumcoeff += coeff;
//    } while (!IncrementIndex(vertex, low, high));
//    if (sumcoeff == 0.0) return Inf;
//    return s / sumcoeff;
//}
/////////////////////////////////////////////////////////////////////////////////////original


Real FMM::Sum(const Vector& x) {
    Real val = 0.0;
    for (int i = 0; i < x.n; i++) val += x(i);
    return val;
}

void FMM::setStateChecker(ob::StateValidityChecker* checker) {
	this->stateChecker = checker;
}

bool FMM::isPathSegmentValid(const Vector& s, const Vector& e) {
    if (normalize(s.size(), s, e) <= cc_resolution)
        return true;
    Vector mid;
    mid.resize(nD);
    for (int i = 0; i < nD; i++) {
        mid[i] = (e[i] + s[i]) / 2;
    }
    if (!isStateValid(mid)){
        this->collision_points.push_back(mid);
        return false;
    }
    return (isPathSegmentValid(s, mid) && isPathSegmentValid(mid, e));
}

bool FMM::isPathValid(const vector< Vector >& path) {
    double t1 = elapsedTime();
    int size = path.size();
    Vector last = (path[0]);
    for (int i = 1; i < size; i++) {
        if (!isStateValid(path[i])) {
            double t2 = elapsedTime();
            this->colcheckTime += t2 - t1;
            this->collision_points.push_back(path[i]);
            return false;
        }
        if (!isPathSegmentValid(last, path[i])) {
            double t2 = elapsedTime();
            this->colcheckTime += t2 - t1;
            return false;
        }
        last = path[i];
    }
    double t2 = elapsedTime();
    this->colcheckTime += t2 - t1;
    return true;
}

bool FMM::isStateValid(const Vector& state) {
	if(thin_of_narrow_passage > 0){
		if(state[0] < 0 && (state[state.size()-1] < (0.5 + thin_of_narrow_passage/2.0) && state[state.size()-1] > (0.5-thin_of_narrow_passage/2.0) ))
			return false;
	}
    ob::State *omplstate = ( (si_->getStateSpace())->as<ob::RealVectorStateSpace>()->allocState() );
    for(int i = 0; i < nD; i++)
    {
    	(omplstate->as<ob::RealVectorStateSpace::StateType>())->values[i] = state[i];
    }

    bool result = stateChecker->isValid(omplstate);
	(si_->getStateSpace())->as<ob::RealVectorStateSpace>()->freeState(omplstate);
	return result;
}


//vector<Vector> FMM::Descend(const ArrayND<Real>& field, const Vector& start) {
//    Vector tmpStart = start;
//    vector<Vector> path;
//    while (true) {
//        Vector grad = FiniteDifference(field, tmpStart);
////        cout<<"grad:"<<grad<<endl;getchar();
//        path.push_back(tmpStart);
//        Real maxval = grad.maxAbsElement();
//        if (maxval == 0)
//            return path;
//        grad *= 1.0 / maxval;
//        Vector next = tmpStart - grad;
//        for (size_t i = 0; i < next.size(); i++) {
//            if (next[i] < 0.0) next[i] = 0.0;
//            if (next[i] >= field.dims[i]) next[i] = field.dims[i] - 1;
//        }
//        if (EvalMultilinear(field, next) >= EvalMultilinear(field, tmpStart))
//            return path;
//        tmpStart = next;
//    }
//    return path;
//}
