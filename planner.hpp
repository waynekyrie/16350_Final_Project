#pragma once
#include "pcb_control/utilities/robot_interface.hpp"

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

using namespace utility;
using namespace std::chrono;
using namespace std;

struct cell{
    Point6D parent;
    double f;
    double g;
    double h;
    cell():parent{-1, -1, -1, -1, -1, -1}, f(-1), g(-1), h(-1){
    }
};


class SamplingPlanner
{
    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        stack<Point6D> searchPath; // Output Trajectory
        vector<Point6D> sampledPts;
        vector<Point6D> backexpandedPts;
        vector<Point6D> traversedPts;

        unordered_map<int, vector<Point6D>> samplePtsConnections; // Idx: [Pt1, Pt2, ... ]
        unordered_map<int, Point6D> idxToPoint; // Idx: Pt
        vector<Point6D> hasIdxPts; // Points that have idx assigned

        unordered_map<int, cell> cellInfo; // idx: cell struct
        unordered_map<int, cell> backCellInfo;

        priority_queue<F_Point6D, vector<F_Point6D>, greater<F_Point6D>> openList; // f: Point6D
        priority_queue<F_Point6D, vector<F_Point6D>, greater<F_Point6D>> backOpenList;
        
        unordered_map<int, bool> closedList; // idx: bool
        unordered_map<int, bool> openListPt;
        unordered_map<int, bool> backOpenListPt;
        unordered_map<int, bool> backClosedList;
        unordered_map<int, bool> updatedList;
        
        /* For all planners */
        int ROBOT_DOF = 6;
        int CONNECTION_RES = 20; // Interpolation resolution to check motion valid
        double REACH_THRESH = 3; // Goal area
        double SAMPLE_EPS = 1.5; // Sampling area
        int NUM_CONNECT_SAMPLES = 8; // Num of samples

        /* PRM */
        int PRM_NUM_NEIGHBORS = 8;
        int PRM_NUM_SAMPLES = 300;

        Point6D start_; // Start config
        Point6D goal_; // Goal Config
        double EQUAL_TOL = 0.0001;

        /* Robot DH Params */
        vector<double> DH_th{0, 0, 0, 0, 0, 0, 0, 0, 0};
        vector<double> DH_d{0.1, 0.1273, 0.220941, 0, -0.1719, 0, 0.1149, 0.1157, 0.3};
        vector<double> DH_a{0, 0, 0, 0.612, 0, 0.5723, 0, 0, 0};
        vector<double> DH_alpha{0, -1.570796, 0, 0, 0, 0, -1.570796, 1.570796, 0};
        Eigen::Matrix<double, 9, 3> ROBOT_POSE;

        /* Obstacles (x, y, z, lx, ly, lz) */
        vector<double> OBS1{0, 1.5, 0.6, 2, 0.1, 1};
        vector<double> OBS2{-1, 0, 0.6, 0.1, 2.8, 1};
        vector<double> OBS3{0, -1.5, 0.6, 2, 0.1, 1};
        vector<double> OBS4{0.9, 0.7, 1, 0.1, 0.1, 1.8};

        /* Robot Joint Limits */
        vector<double> J1_LIMIT{-PI, PI};
        vector<double> J2_LIMIT{-PI, 0.1};
        vector<double> J3_LIMIT{-PI, PI};
        vector<double> J4_LIMIT{-PI, PI};
        vector<double> J5_LIMIT{-PI, PI};
        vector<double> J6_LIMIT{-PI, PI};

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:
        void refreshInternal();
        double distBetweenConfigs(Point6D p1, Point6D p2);
        int getIndex(Point6D pt);
        bool validP2P(Point6D p1, Point6D p2, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        bool validConfig(Point6D config, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        Point6D furthestApproachablePt(Point6D start_pt, Point6D end_pt, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        bool equalPt(Point6D A, Point6D B);
        bool goalReached(Point6D pt, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        bool reachedPt(Point6D p1, Point6D p2, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        Point6D reachedPtInList(Point6D pt, 
                                priority_queue<F_Point6D, vector<F_Point6D>, greater<F_Point6D>> list_pt, 
                                const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        Point6D closestPtInList(Point6D pt, priority_queue<F_Point6D, vector<F_Point6D>, greater<F_Point6D>> list_pt);
        void updateGraph(vector<Point6D> expanded_nodes, 
                         Point6D new_pt, 
                         priority_queue<F_Point6D, vector<F_Point6D>, greater<F_Point6D>> open_list, 
                         const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        void backTrack();
        void backTrackRRTConnect(Point6D mutual_pt);
        void preprocessingSamples(const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        void PRMSearch(const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        void calcFK(Point6D pt, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        bool collisionFreeCartesian(vector<double> pt);

    public:
        SamplingPlanner();
        stack<Point6D> plannerPRM(Point6D start, Point6D goal, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        stack<Point6D> plannerRRT(Point6D start, Point6D goal, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        stack<Point6D> plannerRRTConnect(Point6D start, Point6D goal, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
        stack<Point6D> plannerRRTStar(Point6D start, Point6D goal, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr);
};


