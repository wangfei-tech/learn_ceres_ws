#ifndef G2O_SIMULATOR_H
#define G2O_SIMULATOR_H

#include <map>
#include <vector>

#include "g2o_slam2d_api.h"
#include "se2.h"

namespace g2o
{
    namespace slam2d
    {

        class G2O_SLAM2D_API Simulator
        {
        public:
            enum G2O_SLAM2D_API MotionType
            {
                MO_LEFT,
                MO_RIGHT,
                MO_NUM_ELEMS
            };

            /**
             * \brief simulated landmark
             */
            struct G2O_SLAM2D_API Landmark
            {
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                int id;
                Eigen::Vector2d truePose;
                Eigen::Vector2d simulatedPose;
                std::vector<int> seenBy;
                Landmark() : id(-1) {}
            };
            using LandmarkVector = std::vector<Landmark>;
            using LandmarkPtrVector = std::vector<Landmark *>;

            /**
             * simulated pose of the robot
             */
            struct G2O_SLAM2D_API GridPose
            {
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                int id;
                SE2 truePose;
                SE2 simulatorPose;
                LandmarkPtrVector landmarks; ///< the landmarks observed by this node
            };
            using PosesVector = std::vector<GridPose>;

            /**
             * \brief odometry constraint
             */
            struct G2O_SLAM2D_API GridEdge
            {
                int from;
                int to;
                SE2 trueTransf;
                SE2 simulatorTransf;
                Eigen::Matrix3d information;
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            };
            using GridEdgeVector = std::vector<GridEdge>;

            struct G2O_SLAM2D_API LandmarkEdge
            {
                int from;
                int to;
                Eigen::Vector2d trueMeas;
                Eigen::Vector2d simulatorMeas;
                Eigen::Matrix2d information;
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            };
            using LandmarkEdgeVector = std::vector<LandmarkEdge>;

        public:
            Simulator();
            ~Simulator();

            void simulate(int numPoses, const SE2 &sensorOffset = SE2());

            const PosesVector &poses() const { return _poses; }
            const LandmarkVector &landmarks() const { return _landmarks; }
            const GridEdgeVector &odometry() const { return _odometry; }
            const LandmarkEdgeVector &landmarkObservations() const
            {
                return _landmarkObservations;
            }

        protected:
            PosesVector _poses;
            LandmarkVector _landmarks;
            GridEdgeVector _odometry;
            LandmarkEdgeVector _landmarkObservations;

            GridPose generateNewPose(const GridPose &prev, const SE2 &trueMotion,
                                     const Eigen::Vector2d &transNoise, double rotNoise);
            SE2 getMotion(int motionDirection, double stepLen);
            SE2 sampleTransformation(const SE2 &trueMotion_,
                                     const Eigen::Vector2d &transNoise, double rotNoise);
        };

    } // namespace tutorial
} // namespace g2o

#endif