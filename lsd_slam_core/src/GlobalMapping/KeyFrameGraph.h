/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <vector>
#include <unordered_map>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include "util/EigenCoreInclude.h"
#include <g2o/core/sparse_optimizer.h>
#include "util/SophusUtil.h"
#include "deque"


namespace lsd_slam
{


class Frame;
class KeyFrameGraph;
class VertexSE3;
class EdgeSE3;
class FramePoseStruct;

struct KFConstraintStruct
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	inline KFConstraintStruct()
	{
		firstFrame = secondFrame = 0;
		information.setZero();
		robustKernel = 0;
		edge = 0;

		usage = meanResidual = meanResidualD = meanResidualP = 0;
		reciprocalConsistency = 0;


		idxInAllEdges = -1;
	}

	~KFConstraintStruct();


	Frame* firstFrame;
	Frame* secondFrame;
	Sophus::SE3d secondToFirst;
	Eigen::Matrix<double, 6, 6> information;
	g2o::RobustKernel* robustKernel;
	EdgeSE3* edge;

	float usage;
	float meanResidualD;
	float meanResidualP;
	float meanResidual;

	float reciprocalConsistency;

	int idxInAllEdges;
};

struct GNSSConstraintStruct
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	inline GNSSConstraintStruct()
	{
		information.setZero();
		robustKernel = 0;
		edgeFromWorld = 0;
		edgeFromVehicle = 0;
		graph_added = true;
		timestamp = 0;


		idxInAllEdges = -1;
	}

	~GNSSConstraintStruct();
	
	inline void set_positioning(double positioning[3])
	{
	    longitude = positioning[0];
	    latitude  = positioning[1];
	    altitude  = positioning[2];
	}
	
	inline void set_positioning(double lat, double lon, double alt)
	{
	    longitude = lon;
	    latitude  = lat;
	    altitude  = alt;
	}
	
	inline void set_edgeFromWorld(double enu[3])
	{
		Eigen::Matrix<float, 6, 1> pose;
		pose << enu[0], enu[1], enu[2], 0., 0., 0.;
		vehicleToWorld = SE3::exp(pose.cast<sophusType>());
	}
	
	inline void set_information(double variance[3])
	{
	    information <<
		    variance[0],0.,0.,0.,0.,0.,
		    0.,variance[1],0.,0.,0.,0.,
		    0.,0.,variance[2],0.,0.,0.,
		    0.,0.,0.,9999,0.,0.,
		    0.,0.,0.,0.,9999,0.,
		    0.,0.,0.,0.,0.,9999;
	}
	
	inline void set_information()
	{
		information <<
			0.001,0.,0.,0.,0.,0.,
			0.,0.001,0.,0.,0.,0.,
			0.,0.,0.001,0.,0.,0.,
			0.,0.,0.,0.001,0.,0.,
			0.,0.,0.,0.,0.001,0.,
			0.,0.,0.,0.,0.,0.001;
	}
	
	inline void set_id(int id)
	{
		idInGNSSVertices=id; 
	}

    VertexSE3* worldVertex;
    VertexSE3* vehicleVertex;

	Sophus::SE3d vehicleToWorld;
	Eigen::Matrix<double, 6, 6> information;
	g2o::RobustKernel* robustKernel;
	EdgeSE3* edgeFromWorld;
	EdgeSE3* edgeFromVehicle;
	
	bool graph_added;

	int reciever_id;
	
	double timestamp;

  /** Fix type: 0 means no fix, 1 means 2D fix, 2 means 3D fix, 3 means RTK-float, 4 means RTK-fix */
    int  fix_type;
    
    double longitude;
    double latitude;
    double altitude;
    
    float accuracy_2d;
    float accuracy_3d;
    
    int idInGNSSVertices;
	int idxInAllEdges;
};





/**
 * Graph consisting of KeyFrames and constraints, performing optimization.
 */
class KeyFrameGraph
{
friend class IntegrationTest;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** Constructs an empty pose graph. */
	KeyFrameGraph();

	/** Deletes the g2o graph. */
	~KeyFrameGraph();

	/** Adds a new KeyFrame to the graph. */
	void addKeyFrame(Frame* frame);

	/** Adds a new Frame to the graph. Doesnt actually keep the frame, but only it's pose-struct. */
	void addFrame(Frame* frame);

	void dumpMap(std::string folder);

	/**
	 * Adds a new constraint to the graph.
	 *
	 * The transformation must map world points such that they move as if
	 * attached to a frame which moves from firstFrame to secondFrame:
	 * second->camToWorld * first->worldToCam * point
	 *
	 * If isOdometryConstraint is set, scaleInformation is ignored.
	 */
	void insertConstraint(KFConstraintStruct* constraint);
        void insertConstraint(GNSSConstraintStruct* constraint);
        void insertConstraintOfVehicle2Anntena();

	/** Optimizes the graph. Does not update the keyframe poses,
	 *  only the vertex poses. You must call updateKeyFramePoses() afterwards. */
	int optimize(int num_iterations);
	bool addElementsFromBuffer();


	/**
	 * Creates a hash map of keyframe -> distance to given frame.
	 */
	void calculateGraphDistancesToFrame(Frame* frame, std::unordered_map<Frame*, int>* distanceMap);



	int totalPoints;
	int totalEdges;
	int totalVertices;


	//=========================== Keyframe & Posen Lists & Maps ====================================
	// Always lock the list with the corresponding mutex!
	// central point to administer keyframes, iterate over keyframes, do lookups etc.


	// contains ALL keyframes, as soon as they are "finished".
	// does NOT yet contain the keyframe that is currently being created.
	boost::shared_mutex keyframesAllMutex;
	std::vector< Frame*, Eigen::aligned_allocator<Frame*> > keyframesAll;


	/** Maps frame ids to keyframes. Contains ALL Keyframes allocated, including the one that currently being created. */
	/* this is where the shared pointers of Keyframe Frames are kept, so they are not deleted ever */
	boost::shared_mutex idToKeyFrameMutex;
	std::unordered_map< int, std::shared_ptr<Frame>, std::hash<int>, std::equal_to<int>,
	Eigen::aligned_allocator< std::pair<const int, std::shared_ptr<Frame> > > > idToKeyFrame;


	// contains ALL edges, as soon as they are created
	boost::shared_mutex edgesListsMutex;
	boost::shared_mutex GNSSedgesListsMutex;
	std::vector< KFConstraintStruct*, Eigen::aligned_allocator<KFConstraintStruct*> > edgesAll;
	std::vector< GNSSConstraintStruct*, Eigen::aligned_allocator<GNSSConstraintStruct*> > GNSSedgesAll;



	// contains ALL frame poses, chronologically, as soon as they are tracked.
	// the corresponding frame may have been removed / deleted in the meantime.
	// these are the ones that are also referenced by the corresponding Frame / Keyframe object
	boost::shared_mutex allFramePosesMutex;
	std::vector<FramePoseStruct*, Eigen::aligned_allocator<FramePoseStruct*> > allFramePoses;


	// contains all keyframes in graph, in some arbitrary (random) order. if a frame is re-tracked,
	// it is put to the end of this list; frames for re-tracking are always chosen from the first third of
	// this list.
	boost::mutex keyframesForRetrackMutex;
	std::deque<Frame*> keyframesForRetrack;



private:

	/** Pose graph representation in g2o */
	g2o::SparseOptimizer graph;

	std::vector< Frame*, Eigen::aligned_allocator<Frame*> > newKeyframesBuffer;
	std::vector< KFConstraintStruct*, Eigen::aligned_allocator<FramePoseStruct*> > newEdgeBuffer;
	std::vector< GNSSConstraintStruct*, Eigen::aligned_allocator<GNSSConstraintStruct*> > newGNSSEdgeBuffer;


	int nextEdgeId;
};

}
