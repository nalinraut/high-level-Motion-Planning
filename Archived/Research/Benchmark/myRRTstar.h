/*
 * myRRTstar.h
 *
 *  Created on: Feb 19, 2012
 *      Author: Weiran
 */

#ifndef MYRRTSTAR_H_
#define MYRRTSTAR_H_
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <set>
#include <vector>
#include <cfloat>
#include <list>


namespace ompl
{

    namespace geometric
    {

        class myRRTstar : public base::Planner
        {
        public:

            /** \brief Constructor */
        	myRRTstar(const base::SpaceInformationPtr &si, int iterationLimit) : base::Planner(si, "myRRTstar")
            {
                specs_.approximateSolutions = true;

                goalBias_ = 0.05;
                maxDistance_ = 0.0;
                gamma=1.5;
                lowerBoundMotion=NULL;
                lowerBoundCost=DBL_MAX;

                this->iterationLimit=iterationLimit;
                this->t_rw = 0.0;
                this->t_nnq = 0.0;
                this->t_cc_ = 0;
            }

            virtual ~myRRTstar(void)
            {
                freeMemory();
            }

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias(void) const
            {
                return goalBias_;
            }

            void setCounter(int counter)
            {
            	this->iterationLimit=counter;
            }
            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange(void) const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors data structure */
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Motion*>());
            }

            virtual void setup(void);
            
            double getCCTime()
            {
                return this->t_cc_;
            }

            double getNNQTime()
            {
                return this->t_nnq;
            }

            double getRWTime()
            {
                return this->t_rw;
            }

            void setGama(double gamavalue){
            	this->gamma = gamavalue;
            }



            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:

                Motion(void) : state(NULL), parent(NULL),costFromParent(0.0),costFromRoot(0.0)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL), costFromParent(0.0), costFromRoot(0.0)
                {
                }

                ~Motion(void)
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

                std::set<Motion*> children;

                double 			  costFromParent;

                double  		  costFromRoot;

            };
        private:
            
            //this is a helper function to get the radius of ball near neighbor should be searched within
            double getRadius(void);
            //find the best state in near-neighbor-hood of sampled state and set it as the parent
            bool findBestParent(Motion* rndMotion, std::vector<Motion*>& nbh, Motion*& bestParent);

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion* a, const Motion* b) const
            {
                return si_->distance(a->state, b->state);
            }

            /*int compareCost(std::pair<Motion*,double> i, std::pair<Motion*,double> j)
            {
                return (i.second < j.second);
            }*/
            
            //sample a state from space
            void sampleAState(ompl::base::State* rndState);
            
            //do some preparation for solving
            bool prepareSolve();
            
            //decide whether or not should the state be inserted
            bool shouldInsert(Motion* rndMotion);
            
            //decide whether should interpolate a state between rndMotion and bestParent
            void shouldInterpolate(Motion* bestParent, Motion* rndMotion);
            
            //insert a motion into near-neighbor-hood
            void insertMotion(Motion* bestParent, 
                              Motion*& addedMotion,
                              Motion* rndMotion);
            
            //check if we find a better motion than before
            void checkBetterMotion(Motion* motion, base::Goal* goal);
            
            //rewire if needed
            void rewire(std::vector<Motion*>& nbh, Motion* addedMotion);

            void updateBranchCost(Motion* motion, int level, base::Goal* goal);
            
            //store the found path
            void storePath();
            
            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The random number generator */
            RNG                                            rng_;

            //limit of number of iterations
            int 										   iterationLimit;

            double 										   gamma;

            Motion*										   lowerBoundMotion;

            double 										   lowerBoundCost;
            
            //a timer for rewiring
            double                                         t_rw;
            
            //a timer for nearest neighbor query
            double                                         t_nnq;

			//collision checking time
			double 										   t_cc_;
        };

    }
}



#endif /* MYRRTSTAR_H_ */
