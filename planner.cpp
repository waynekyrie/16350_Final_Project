#include "pcb_control/trackit/planner.hpp"

SamplingPlanner::SamplingPlanner()
{
    ROS_INFO_STREAM("Initialized Planner!");
}

stack<Point6D> SamplingPlanner::plannerPRM(Point6D start, Point6D goal, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr)
{
    cout<<"Running PRM Planner!"<<endl;
    start_ = start;
    goal_ = goal;

    auto start_time = high_resolution_clock::now();
    auto start_time_total = high_resolution_clock::now();
    refreshInternal();
    auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    cout<<"Refresh! Time cost: "<<duration.count()<<"ms"<<endl;

    start_time = high_resolution_clock::now();
    preprocessingSamples(robot_ptr);
    duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    cout<<"Preprocessing! Time cost: "<<duration.count()<<"ms"<<endl;

    start_time = high_resolution_clock::now();
    PRMSearch(robot_ptr);
    duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    cout<<"PRM Search! Time cost: "<<duration.count()<<"ms"<<endl;
    cout<<"Path length: "<<searchPath.size()<<endl;
    duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time_total);
    cout<<"PRM Time cost: "<<duration.count()<<"ms"<<endl;
    return searchPath;
}

stack<Point6D> SamplingPlanner::plannerRRT(Point6D start, Point6D goal, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr)
{
    cout<<"Running RRT Planner!"<<endl;
    start_ = start;
    goal_ = goal;

    auto start_time = high_resolution_clock::now();
    refreshInternal();
    auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    cout<<"Refresh! Time cost: "<<duration.count()<<"ms"<<endl;

    start_time = high_resolution_clock::now();
    Point6D cur_pt = start_;
    int cur_idx = getIndex(cur_pt);
    cellInfo[cur_idx].g = 0;
    cellInfo[cur_idx].h = distBetweenConfigs(cur_pt, goal_);
    cellInfo[cur_idx].f = cellInfo[cur_idx].g + cellInfo[cur_idx].h;
    F_Point6D pt{cellInfo[cur_idx].f, cur_pt[0], cur_pt[1], cur_pt[2], cur_pt[3], cur_pt[4], cur_pt[5]};

    openList.push(pt);
    openListPt[cur_idx] = true;
    srand((unsigned) time(0));
    int count = 1;
     
    while(!openList.empty())
    {
        // Get the point with lowest cost f
        F_Point6D cur_pt_info = openList.top();
        openList.pop();
        cur_pt[0] = cur_pt_info[1];
        cur_pt[1] = cur_pt_info[2];
        cur_pt[2] = cur_pt_info[3];
        cur_pt[3] = cur_pt_info[4];
        cur_pt[4] = cur_pt_info[5];
        cur_pt[5] = cur_pt_info[6];
        cur_idx = getIndex(cur_pt);
        double cur_g = cellInfo[cur_idx].g;

        if(goalReached(cur_pt, robot_ptr)){
            cout<<"reached goal"<<endl;
            int goal_idx = getIndex(goal_);
            cellInfo[goal_idx].parent = cur_pt;
            backTrack();
            duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
            cout<<"RRT Planner time cost: "<<duration.count()<<"ms"<<endl;
            cout<<"Path length: "<<searchPath.size()<<endl;
            cout<<"Expanded Nodes: "<<count<<endl;
            return searchPath;
        }

        if(closedList[cur_idx] == true){
            continue;
        }
        closedList[cur_idx] = true;

        sampledPts.clear();
        while(sampledPts.size() < NUM_CONNECT_SAMPLES){
            Point6D sample_pt = start_;
            double high_b = 0;
            double low_b = 0;
            for(int j=0; j<ROBOT_DOF; j++){
                int randomInt = rand();
                double cur_low = cur_pt[j] - SAMPLE_EPS;
                double cur_high = cur_pt[j] + SAMPLE_EPS;

                while(cur_low < -PI){
                    cur_low = cur_low + 2*PI;
                }
                while(cur_low > PI){
                    cur_low = cur_low - 2*PI;
                }
                while(cur_high < -PI){
                    cur_high = cur_high + 2*PI;
                }
                while(cur_high > PI){
                    cur_high = cur_high - 2*PI;
                }
                
                if(j == 0){
                    low_b = MAX(J1_LIMIT[0], cur_low);
                    high_b = MIN(J1_LIMIT[1], cur_high);
                }
                else if(j == 1){
                    low_b = MAX(J2_LIMIT[0], cur_low);
                    high_b = MIN(J2_LIMIT[1], cur_high);
                }
                else if(j == 2){
                    low_b = MAX(J3_LIMIT[0], cur_low);
                    high_b = MIN(J3_LIMIT[1], cur_high);
                }
                else if(j == 3){
                    low_b = MAX(J4_LIMIT[0], cur_low);
                    high_b = MIN(J4_LIMIT[1], cur_high);
                }
                else if(j == 4){
                    low_b = MAX(J5_LIMIT[0], cur_low);
                    high_b = MIN(J5_LIMIT[1], cur_high);
                }
                else if(j == 5){
                    low_b = MAX(J6_LIMIT[0], cur_low);
                    high_b = MIN(J6_LIMIT[1], cur_high);
                }
                double sample = ((double)randomInt) / RAND_MAX * (high_b-low_b) + low_b;
                sample_pt[j] = sample;
            }
            int sample_idx = getIndex(sample_pt);
            if(!validConfig(sample_pt, robot_ptr) || !validP2P(cur_pt, sample_pt, robot_ptr) || openListPt[sample_idx]){
                continue;
            }
            sampledPts.push_back(sample_pt);
        }

        for(int i = 0; i < sampledPts.size(); i++){
            Point6D next_pt = sampledPts[i];
            int next_idx = getIndex(next_pt);
            Point6D closest_pt = cur_pt;
            int closest_idx = getIndex(closest_pt);
            
            double next_g = cellInfo[closest_idx].g + distBetweenConfigs(closest_pt, next_pt);
            double next_h = distBetweenConfigs(goal_, next_pt);
            double next_f = next_g + next_h;
        
            cellInfo[next_idx].g = next_g;
            cellInfo[next_idx].h = next_h;
            cellInfo[next_idx].f = next_f;
            cellInfo[next_idx].parent = closest_pt;
                            
            F_Point6D fpt{cellInfo[next_idx].f, next_pt[0], next_pt[1], next_pt[2], next_pt[3], next_pt[4], next_pt[5]};
            openList.push(fpt);
            openListPt[next_idx] = true;
            count++;
        }
    }
    duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    cout<<"RRT Planner time cost: "<<duration.count()<<"ms"<<endl;
    cout<<"Path length: "<<searchPath.size()<<endl;
    return searchPath;
}

stack<Point6D> SamplingPlanner::plannerRRTConnect(Point6D start, Point6D goal, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr)
{
    cout<<"Running RRT Connect Planner!"<<endl;
    start_ = start;
    goal_ = goal;

    auto start_time = high_resolution_clock::now();
    refreshInternal();
    auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    cout<<"Refresh! Time cost: "<<duration.count()<<"ms"<<endl;

    start_time = high_resolution_clock::now();
    // Initialize forward graph
    Point6D cur_pt = start_;
    int cur_idx = getIndex(cur_pt);
    cellInfo[cur_idx].g = 0;
    cellInfo[cur_idx].h = distBetweenConfigs(cur_pt, goal_);
    cellInfo[cur_idx].f = cellInfo[cur_idx].g + cellInfo[cur_idx].h;
    F_Point6D pt{cellInfo[cur_idx].f, cur_pt[0], cur_pt[1], cur_pt[2], cur_pt[3], cur_pt[4], cur_pt[5]};
    openList.push(pt);
    openListPt[cur_idx] = true;

    // Initialize backward graph
    cur_pt = goal_;
    cur_idx = getIndex(cur_pt);
    backCellInfo[cur_idx].g = 0;
    backCellInfo[cur_idx].h = distBetweenConfigs(cur_pt, start_);
    backCellInfo[cur_idx].f = backCellInfo[cur_idx].g + backCellInfo[cur_idx].h;
    F_Point6D back_pt{backCellInfo[cur_idx].f, cur_pt[0], cur_pt[1], cur_pt[2], cur_pt[3], cur_pt[4], cur_pt[5]};
    backOpenList.push(back_pt);
    backOpenListPt[cur_idx] = true;

    bool forward = true;
    srand((unsigned) time(0));
    int count = 2;

    while(1)
    {
        if(forward)
        {
            F_Point6D cur_pt_info = openList.top();
            openList.pop();
            cur_pt[0] = cur_pt_info[1];
            cur_pt[1] = cur_pt_info[2];
            cur_pt[2] = cur_pt_info[3];
            cur_pt[3] = cur_pt_info[4];
            cur_pt[4] = cur_pt_info[5];
            cur_pt[5] = cur_pt_info[6];
            cur_idx = getIndex(cur_pt);
            double cur_g = cellInfo[cur_idx].g;

            Point6D reached_pt = reachedPtInList(cur_pt, backOpenList, robot_ptr);
            if(!equalPt(cur_pt, reached_pt)){
                cout<<"Reached goal from forward"<<endl;
                int reached_idx = getIndex(reached_pt);
                cellInfo[reached_idx].parent = cur_pt;
                backTrackRRTConnect(reached_pt);
                duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
                cout<<"RRT Connect Search! Time cost: "<<duration.count()<<"ms"<<endl;
                cout<<"Path length: "<<searchPath.size()<<endl;
                cout<<"Expanded Nodes: "<<count<<endl;
                return searchPath;
            }

            if(closedList[cur_idx]){
                forward = false;
                continue;
            }
            closedList[cur_idx] = true;

            sampledPts.clear();
            while(sampledPts.size() < NUM_CONNECT_SAMPLES){
                Point6D sample_pt = cur_pt;
                double high_b = 0;
                double low_b = 0;
                for(int j=0; j<ROBOT_DOF; j++){
                    int randomInt = rand();
                    double cur_low = cur_pt[j] - SAMPLE_EPS;
                    double cur_high = cur_pt[j] + SAMPLE_EPS;

                    while(cur_low < -PI){
                        cur_low = cur_low + 2*PI;
                    }
                    while(cur_low > PI){
                        cur_low = cur_low - 2*PI;
                    }
                    while(cur_high < -PI){
                        cur_high = cur_high + 2*PI;
                    }
                    while(cur_high > PI){
                        cur_high = cur_high - 2*PI;
                    }
                    
                    if(j == 0){
                        low_b = MAX(J1_LIMIT[0], cur_low);
                        high_b = MIN(J1_LIMIT[1], cur_high);
                    }
                    else if(j == 1){
                        low_b = MAX(J2_LIMIT[0], cur_low);
                        high_b = MIN(J2_LIMIT[1], cur_high);
                    }
                    else if(j == 2){
                        low_b = MAX(J3_LIMIT[0], cur_low);
                        high_b = MIN(J3_LIMIT[1], cur_high);
                    }
                    else if(j == 3){
                        low_b = MAX(J4_LIMIT[0], cur_low);
                        high_b = MIN(J4_LIMIT[1], cur_high);
                    }
                    else if(j == 4){
                        low_b = MAX(J5_LIMIT[0], cur_low);
                        high_b = MIN(J5_LIMIT[1], cur_high);
                    }
                    else if(j == 5){
                        low_b = MAX(J6_LIMIT[0], cur_low);
                        high_b = MIN(J6_LIMIT[1], cur_high);
                    }
                    double sample = ((double)randomInt) / RAND_MAX * (high_b-low_b) + low_b;
                    sample_pt[j] = sample;
                }
                int sample_idx = getIndex(sample_pt);
                if(!validConfig(sample_pt, robot_ptr) || openListPt[sample_idx]){
                    continue;
                }
                sampledPts.push_back(sample_pt);
            }
        
            for(int i = 0; i < sampledPts.size(); i++){
                Point6D next_pt = furthestApproachablePt(cur_pt, sampledPts[i], robot_ptr);
                if(equalPt(next_pt, cur_pt)){
                    continue;
                }
                int next_idx = getIndex(next_pt);
                double next_g = cellInfo[cur_idx].g + distBetweenConfigs(cur_pt, next_pt);
                double next_h = distBetweenConfigs(goal_, next_pt);
                double next_f = next_g + next_h;
        
                cellInfo[next_idx].g = next_g;
                cellInfo[next_idx].h = next_h;
                cellInfo[next_idx].f = next_f;
                cellInfo[next_idx].parent = cur_pt;
                            
                F_Point6D fpt{cellInfo[next_idx].f, next_pt[0], next_pt[1], next_pt[2], next_pt[3], next_pt[4], next_pt[5]};
                openList.push(fpt);
                openListPt[next_idx] = true;
                count++;
                // Forward tree reached to random sample, approach from backward tree
                if(equalPt(next_pt, sampledPts[i])){
                    Point6D cur_back_pt = closestPtInList(next_pt, backOpenList);
                    if(equalPt(cur_back_pt, next_pt)){
                        continue;
                    }
                    int cur_back_idx = getIndex(cur_back_pt);
                    Point6D back_next_pt = furthestApproachablePt(cur_back_pt, next_pt, robot_ptr);
                    // Cannot move
                    if(equalPt(back_next_pt, cur_back_pt)){
                        continue;
                    }
                    double back_next_idx = getIndex(back_next_pt);
                    double back_next_g = backCellInfo[cur_back_idx].g + distBetweenConfigs(cur_back_pt, back_next_pt);
                    double back_next_h = distBetweenConfigs(start_, back_next_pt);
                    double back_next_f = back_next_g + back_next_h;
                
                    backCellInfo[back_next_idx].g = back_next_g;
                    backCellInfo[back_next_idx].h = back_next_h;
                    backCellInfo[back_next_idx].f = back_next_f;
                    backCellInfo[back_next_idx].parent = cur_back_pt;
                                    
                    F_Point6D back_fpt{backCellInfo[back_next_idx].f, back_next_pt[0], back_next_pt[1], back_next_pt[2], back_next_pt[3], back_next_pt[4], back_next_pt[5]};
                    backOpenList.push(back_fpt);
                    count++;
                    backOpenListPt[back_next_idx] = true;
                }
            }
            forward = false;
        }
        else{
            F_Point6D cur_pt_info = backOpenList.top();
            backOpenList.pop();
            cur_pt[0] = cur_pt_info[1];
            cur_pt[1] = cur_pt_info[2];
            cur_pt[2] = cur_pt_info[3];
            cur_pt[3] = cur_pt_info[4];
            cur_pt[4] = cur_pt_info[5];
            cur_pt[5] = cur_pt_info[6];
            cur_idx = getIndex(cur_pt);
            double cur_g = backCellInfo[cur_idx].g;

            Point6D reached_pt = reachedPtInList(cur_pt, openList, robot_ptr);
            if(!equalPt(cur_pt, reached_pt)){
                cout<<"Reached goal from backward"<<endl;
                int reached_idx = getIndex(reached_pt);
                backCellInfo[reached_idx].parent = cur_pt;
                backTrackRRTConnect(reached_pt);
                duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
                cout<<"RRT Connect Search! Time cost: "<<duration.count()<<"ms"<<endl;
                cout<<"Path length: "<<searchPath.size()<<endl;
                cout<<"Expanded Nodes: "<<count<<endl;
                return searchPath;
            }

            if(backClosedList[cur_idx]){
                forward = true;
                continue;
            }
            backClosedList[cur_idx] = true;

            sampledPts.clear();
            while(sampledPts.size() < NUM_CONNECT_SAMPLES){
                Point6D sample_pt = cur_pt;
                double high_b = 0;
                double low_b = 0;
                for(int j=0; j<ROBOT_DOF; j++){
                    int randomInt = rand();
                    double cur_low = cur_pt[j] - SAMPLE_EPS;
                    double cur_high = cur_pt[j] + SAMPLE_EPS;

                    while(cur_low < -PI){
                        cur_low = cur_low + 2*PI;
                    }
                    while(cur_low > PI){
                        cur_low = cur_low - 2*PI;
                    }
                    while(cur_high < -PI){
                        cur_high = cur_high + 2*PI;
                    }
                    while(cur_high > PI){
                        cur_high = cur_high - 2*PI;
                    }
                    
                    if(j == 0){
                        low_b = MAX(J1_LIMIT[0], cur_low);
                        high_b = MIN(J1_LIMIT[1], cur_high);
                    }
                    else if(j == 1){
                        low_b = MAX(J2_LIMIT[0], cur_low);
                        high_b = MIN(J2_LIMIT[1], cur_high);
                    }
                    else if(j == 2){
                        low_b = MAX(J3_LIMIT[0], cur_low);
                        high_b = MIN(J3_LIMIT[1], cur_high);
                    }
                    else if(j == 3){
                        low_b = MAX(J4_LIMIT[0], cur_low);
                        high_b = MIN(J4_LIMIT[1], cur_high);
                    }
                    else if(j == 4){
                        low_b = MAX(J5_LIMIT[0], cur_low);
                        high_b = MIN(J5_LIMIT[1], cur_high);
                    }
                    else if(j == 5){
                        low_b = MAX(J6_LIMIT[0], cur_low);
                        high_b = MIN(J6_LIMIT[1], cur_high);
                    }
                    double sample = ((double)randomInt) / RAND_MAX * (high_b-low_b) + low_b;
                    sample_pt[j] = sample;
                }
                int sample_idx = getIndex(sample_pt);
                if(!validConfig(sample_pt, robot_ptr) || openListPt[sample_idx]){
                    continue;
                }
                sampledPts.push_back(sample_pt);
            }

            for(int i = 0; i < sampledPts.size(); i++){
                Point6D next_pt = furthestApproachablePt(cur_pt, sampledPts[i], robot_ptr);
                if(equalPt(next_pt, cur_pt)){
                    continue;
                }
                int next_idx = getIndex(next_pt);
                double next_g = backCellInfo[cur_idx].g + distBetweenConfigs(cur_pt, next_pt);
                double next_h = distBetweenConfigs(start_, next_pt);
                double next_f = next_g + next_h;
        
                backCellInfo[next_idx].g = next_g;
                backCellInfo[next_idx].h = next_h;
                backCellInfo[next_idx].f = next_f;
                backCellInfo[next_idx].parent = cur_pt;
                            
                F_Point6D fpt{backCellInfo[next_idx].f, next_pt[0], next_pt[1], next_pt[2], next_pt[3], next_pt[4], next_pt[5]};
                backOpenList.push(fpt);
                backOpenListPt[next_idx] = true;
                count++;

                if(equalPt(next_pt, sampledPts[i])){
                    Point6D cur_back_pt = closestPtInList(next_pt, openList);
                    if(equalPt(cur_back_pt, next_pt)){
                        continue;
                    }
                    int cur_back_idx = getIndex(cur_back_pt);
                    Point6D back_next_pt = furthestApproachablePt(cur_back_pt, next_pt, robot_ptr);
                    if(equalPt(back_next_pt, cur_back_pt)){
                        continue;
                    }
                    int back_next_idx = getIndex(back_next_pt);
                    double back_next_g = cellInfo[cur_back_idx].g + distBetweenConfigs(cur_back_pt, back_next_pt);
                    double back_next_h = distBetweenConfigs(goal_, back_next_pt);
                    double back_next_f = back_next_g + back_next_h;
                
                    cellInfo[back_next_idx].g = back_next_g;
                    cellInfo[back_next_idx].h = back_next_h;
                    cellInfo[back_next_idx].f = back_next_f;
                    cellInfo[back_next_idx].parent = cur_back_pt;
                                    
                    F_Point6D back_fpt{cellInfo[back_next_idx].f, back_next_pt[0], back_next_pt[1], back_next_pt[2], back_next_pt[3], back_next_pt[4], back_next_pt[5]};
                    openList.push(back_fpt);
                    openListPt[back_next_idx] = true;
                    count++;
                }
            }
            forward = true;
        }
    }
}


stack<Point6D> SamplingPlanner::plannerRRTStar(Point6D start, Point6D goal, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr)
{
    cout<<"Running RRT Star Planner!"<<endl;
    start_ = start;
    goal_ = goal;

    auto start_time = high_resolution_clock::now();
    refreshInternal();
    auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    cout<<"Refresh! Time cost: "<<duration.count()<<"ms"<<endl;

    start_time = high_resolution_clock::now();

    Point6D cur_pt = start_;
    int cur_idx = getIndex(cur_pt);
    
    cellInfo[cur_idx].g = 0;
    cellInfo[cur_idx].h = distBetweenConfigs(cur_pt, goal_);
    cellInfo[cur_idx].f = cellInfo[cur_idx].g + cellInfo[cur_idx].h;
    F_Point6D pt{cellInfo[cur_idx].f, cur_pt[0], cur_pt[1], cur_pt[2], cur_pt[3], cur_pt[4], cur_pt[5]};

    openList.push(pt);
    openListPt[cur_idx] = true;
    traversedPts.push_back(cur_pt);
    int count = 1;
     
    while(!openList.empty())
    {
        // Get the point with lowest cost f
        F_Point6D cur_pt_info = openList.top();
        openList.pop();
        cur_pt[0] = cur_pt_info[1];
        cur_pt[1] = cur_pt_info[2];
        cur_pt[2] = cur_pt_info[3];
        cur_pt[3] = cur_pt_info[4];
        cur_pt[4] = cur_pt_info[5];
        cur_pt[5] = cur_pt_info[6];
        cur_idx = getIndex(cur_pt);
        double cur_g = cellInfo[cur_idx].g;

        if(goalReached(cur_pt, robot_ptr)){
            int goal_idx = getIndex(goal_);
            cellInfo[goal_idx].parent = cur_pt;
            backTrack();
            duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
            cout<<"RRT Star Search Time cost: "<<duration.count()<<"ms"<<endl;
            cout<<"Path length: "<<searchPath.size()<<endl;
            cout<<"Expanded Nodes: "<<count<<endl;
            return searchPath;
        }

        if(closedList[cur_idx]){
            continue;
        }
        closedList[cur_idx] = true;

        sampledPts.clear();
        while(sampledPts.size() < NUM_CONNECT_SAMPLES){
            Point6D sample_pt = cur_pt;
            double high_b = 0;
            double low_b = 0;
            for(int j=0; j<ROBOT_DOF; j++){
                int randomInt = rand();
                double cur_low = cur_pt[j] - SAMPLE_EPS;
                double cur_high = cur_pt[j] + SAMPLE_EPS;

                while(cur_low < -PI){
                    cur_low = cur_low + 2*PI;
                }
                while(cur_low > PI){
                    cur_low = cur_low - 2*PI;
                }
                while(cur_high < -PI){
                    cur_high = cur_high + 2*PI;
                }
                while(cur_high > PI){
                    cur_high = cur_high - 2*PI;
                }
                    
                if(j == 0){
                    low_b = MAX(J1_LIMIT[0], cur_low);
                    high_b = MIN(J1_LIMIT[1], cur_high);
                }
                else if(j == 1){
                    low_b = MAX(J2_LIMIT[0], cur_low);
                    high_b = MIN(J2_LIMIT[1], cur_high);
                }
                else if(j == 2){
                    low_b = MAX(J3_LIMIT[0], cur_low);
                    high_b = MIN(J3_LIMIT[1], cur_high);
                }
                else if(j == 3){
                    low_b = MAX(J4_LIMIT[0], cur_low);
                    high_b = MIN(J4_LIMIT[1], cur_high);
                }
                else if(j == 4){
                    low_b = MAX(J5_LIMIT[0], cur_low);
                    high_b = MIN(J5_LIMIT[1], cur_high);
                }
                else if(j == 5){
                    low_b = MAX(J6_LIMIT[0], cur_low);
                    high_b = MIN(J6_LIMIT[1], cur_high);
                }
                double sample = ((double)randomInt) / RAND_MAX * (high_b-low_b) + low_b;
                sample_pt[j] = sample;
            }
            int sample_idx = getIndex(sample_pt);
            if(!validConfig(sample_pt, robot_ptr) || !validP2P(cur_pt, sample_pt, robot_ptr) || openListPt[sample_idx] || closedList[sample_idx]){
                continue;
            }
            sampledPts.push_back(sample_pt);
        }
        
        for(int i = 0; i < sampledPts.size(); i++){
            Point6D next_pt = sampledPts[i];
            int next_idx = getIndex(next_pt);
            
            Point6D closest_pt = cur_pt;
            int closest_idx = getIndex(closest_pt);
            
            double next_g = cellInfo[closest_idx].g + distBetweenConfigs(closest_pt, next_pt);
            double next_h = distBetweenConfigs(goal_, next_pt);
            double next_f = next_g + next_h;
        
            cellInfo[next_idx].g = next_g;
            cellInfo[next_idx].h = next_h;
            cellInfo[next_idx].f = next_f;
            cellInfo[next_idx].parent = closest_pt;
            
            F_Point6D fpt{cellInfo[next_idx].f, next_pt[0], next_pt[1], next_pt[2], next_pt[3], next_pt[4], next_pt[5]};
            openList.push(fpt);
            count++;
            openListPt[next_idx] = true;
            traversedPts.push_back(next_pt);
            updateGraph(traversedPts, next_pt, openList, robot_ptr);
        }
    }
    duration = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    cout<<"RRT Star Search Time cost: "<<duration.count()<<"ms"<<endl;
    cout<<"Path length: "<<searchPath.size()<<endl;
    return searchPath;
}


void SamplingPlanner::updateGraph(vector<Point6D> expanded_nodes, 
                                  Point6D new_pt, 
                                  priority_queue<F_Point6D, vector<F_Point6D>, greater<F_Point6D>> open_list,
                                  const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr){
    int count = 0;
    while(!open_list.empty()){
        count ++;
        if(count >= 25){
            break;
        }
        F_Point6D cur_pt_info = open_list.top();
        open_list.pop();
        Point6D cur_pt{cur_pt_info[1], cur_pt_info[2], cur_pt_info[3], cur_pt_info[4], cur_pt_info[5], cur_pt_info[6]};
        int cur_idx = getIndex(cur_pt);
        
        if(distBetweenConfigs(cur_pt, new_pt) > 0.2 || updatedList[cur_idx]){
            continue;
        }
        updatedList[cur_idx] = true;

        double original_f = cellInfo[cur_idx].f;
        for(int i=0; i<expanded_nodes.size(); i++){
            Point6D potential_parent = expanded_nodes[i];
            if(!validP2P(potential_parent, cur_pt, robot_ptr)){
                continue;
            }
            int potential_parent_idx = getIndex(potential_parent);

            double g = cellInfo[potential_parent_idx].g + distBetweenConfigs(potential_parent, cur_pt);
            double h = distBetweenConfigs(goal_, cur_pt);
            double f = g + h;
            if(f < original_f){
                cellInfo[cur_idx].g = g;
                cellInfo[cur_idx].h = h;
                cellInfo[cur_idx].f = f;
                cellInfo[cur_idx].parent = potential_parent;
                original_f = f;
            }
        }
    }
}

void SamplingPlanner::preprocessingSamples(const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr)
{
    srand((unsigned) time(0));
    while(sampledPts.size() < PRM_NUM_SAMPLES){
        Point6D sample_pt = start_;
        double high_b = 0;
        double low_b = 0;
        for(int j=0; j<ROBOT_DOF; j++){
            int randomInt = rand();
            if(j == 0){
                low_b = J1_LIMIT[0];
                high_b = J1_LIMIT[1];
            }
            else if(j == 1){
                low_b = J2_LIMIT[0];
                high_b = J2_LIMIT[1];
            }
            else if(j == 2){
                low_b = J3_LIMIT[0];
                high_b = J3_LIMIT[1];
            }
            else if(j == 3){
                low_b = J4_LIMIT[0];
                high_b = J4_LIMIT[1];
            }
            else if(j == 4){
                low_b = J5_LIMIT[0];
                high_b = J5_LIMIT[1];
            }
            else if(j == 5){
                low_b = J6_LIMIT[0];
                high_b = J6_LIMIT[1];
            }
            double sample = ((double)randomInt) / RAND_MAX * (high_b-low_b) + low_b;
            sample_pt[j] = sample;
        }
        double sample_idx = getIndex(sample_pt);
        if(!validConfig(sample_pt, robot_ptr) || openListPt[sample_idx]){
            continue;
        }
        sampledPts.push_back(sample_pt);
        openListPt[sample_idx] = true;
    }

    sampledPts.push_back(start_);
    sampledPts.push_back(goal_);
  
    // Build graph
    for(int i=0; i<sampledPts.size(); i++){
        Point6D cur_pt = sampledPts[i];
        std::map<double, Point6D> priority_q;
        for(int j=0; j<sampledPts.size(); j++){
            Point6D next_pt = sampledPts[j];
            if(i != j){
                double dist = distBetweenConfigs(cur_pt, next_pt);
                priority_q[dist] = next_pt;
            }
        }
        int cur_idx = getIndex(cur_pt);
        for(auto j=priority_q.begin(); j!=priority_q.end(); j++){
            if(samplePtsConnections[cur_idx].size() >= PRM_NUM_NEIGHBORS){
                break;
            }
            Point6D next_pt = j->second;
            if(validP2P(cur_pt, next_pt, robot_ptr)){
                int neighbor_idx = getIndex(next_pt);
                if(samplePtsConnections[neighbor_idx].size() < PRM_NUM_NEIGHBORS){
                    samplePtsConnections[cur_idx].push_back(next_pt);
                    samplePtsConnections[neighbor_idx].push_back(cur_pt);
                }
            }
        }
    }
}

void SamplingPlanner::PRMSearch(const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr){
    Point6D cur_pt = start_;
    int cur_idx = getIndex(cur_pt);
    cellInfo[cur_idx].g = 0;
    cellInfo[cur_idx].h = distBetweenConfigs(cur_pt, goal_);
    cellInfo[cur_idx].f = cellInfo[cur_idx].g + cellInfo[cur_idx].h;
    F_Point6D pt{cellInfo[cur_idx].f, cur_pt[0], cur_pt[1], cur_pt[2], cur_pt[3], cur_pt[4], cur_pt[5]};
    openList.push(pt);
  
    int count = 1;
    while(!openList.empty())
    {
        // Get the point with lowest cost f
        F_Point6D cur_pt_info = openList.top();
        openList.pop();
        // expandedNodes.push_back(cur_pt);
        cur_pt[0] = cur_pt_info[1];
        cur_pt[1] = cur_pt_info[2];
        cur_pt[2] = cur_pt_info[3];
        cur_pt[3] = cur_pt_info[4];
        cur_pt[4] = cur_pt_info[5];
        cur_pt[5] = cur_pt_info[6];
        cur_idx = getIndex(cur_pt);
        double cur_g = cellInfo[cur_idx].g;

        if(equalPt(cur_pt, goal_)){
            backTrack();
            cout<<"PRM Expanded Nodes: "<<count<<endl;
            return;
        }

        if(closedList[cur_idx]){
            continue;
        }
        closedList[cur_idx] = true;

        for(int i = 0; i < samplePtsConnections[cur_idx].size(); i++){
            Point6D next_pt = samplePtsConnections[cur_idx][i];
            int next_idx = getIndex(next_pt);
            
            // If next step is valid
            if(validConfig(next_pt, robot_ptr)){
                double next_g = cur_g + distBetweenConfigs(cur_pt, next_pt);
                double next_h = distBetweenConfigs(next_pt, goal_);
                double next_f = next_g + next_h;

                // If have not been searched before or previous search had a higher cost value
                if(cellInfo[next_idx].f == -1 || cellInfo[next_idx].f > next_f){
                    cellInfo[next_idx].g = next_g;
                    cellInfo[next_idx].h = next_h;
                    cellInfo[next_idx].f = next_f;
                    cellInfo[next_idx].parent = cur_pt;
                                
                    F_Point6D pt{cellInfo[next_idx].f, next_pt[0], next_pt[1], next_pt[2], next_pt[3], next_pt[4], next_pt[5]};
                    openList.push(pt);
                    count ++;
                }
            }
        }
    }
}


void SamplingPlanner::refreshInternal(){
    while(!searchPath.empty()){
        searchPath.pop();
    }
    sampledPts.clear();
    backexpandedPts.clear();

    samplePtsConnections.clear();
    idxToPoint.clear();
    hasIdxPts.clear();
    cellInfo.clear();
    backCellInfo.clear();

    while(!openList.empty()){
        openList.pop();
    }
    while(!backOpenList.empty()){
        backOpenList.pop();
    }
    closedList.clear();
    openListPt.clear();
    backClosedList.clear();
    updatedList.clear();
}

int SamplingPlanner::getIndex(Point6D pt){
    for(int i=0; i<idxToPoint.size(); i++){
        if(equalPt(pt, idxToPoint[i])){
            return i;
        }
    }
    int pt_idx = idxToPoint.size();
    idxToPoint[pt_idx] = pt;
    hasIdxPts.push_back(pt);
    return pt_idx;
}

double SamplingPlanner::distBetweenConfigs(Point6D p1, Point6D p2){
    double h = 0;
    for(int i=0; i<ROBOT_DOF; i++){
        h = h + pow(p1[i] - p2[i], 2);
    }
    return sqrt(h);
}

bool SamplingPlanner::validP2P(Point6D p1, Point6D p2, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr){
    Point6D dist{p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2], p2[3] - p1[3], p2[4] - p1[4], p2[5] - p1[5]};
    Point6D interval{dist[0]/CONNECTION_RES, dist[1]/CONNECTION_RES, dist[2]/CONNECTION_RES, 
                     dist[3]/CONNECTION_RES, dist[4]/CONNECTION_RES, dist[5]/CONNECTION_RES};
    for(int i=0; i<=CONNECTION_RES; i++){
        Point6D pt{p1[0]+i*interval[0], p1[1]+i*interval[1], p1[2]+i*interval[2], p1[3]+i*interval[3], p1[4]+i*interval[4], p1[5]+i*interval[5]};
        if(!validConfig(pt, robot_ptr)){
            return false;
        }
    }
    return true;
}

bool SamplingPlanner::validConfig(Point6D pt, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr){
    calcFK(pt, robot_ptr);
    for(int i=0; i<8; i++){
        vector<double> start_pose{ROBOT_POSE(i, 0), ROBOT_POSE(i, 1), ROBOT_POSE(i, 2)};
        vector<double> end_pose{ROBOT_POSE(i+1, 0), ROBOT_POSE(i+1, 1), ROBOT_POSE(i+1, 2)};
    
        vector<double> dist{end_pose[0] - start_pose[0], end_pose[1] - start_pose[1], end_pose[2] - start_pose[2]};
        vector<double> interval{dist[0]/CONNECTION_RES, dist[1]/CONNECTION_RES, dist[2]/CONNECTION_RES};    
        for(int j=0; j<=CONNECTION_RES; j++){
            vector<double> robot_config{start_pose[0]+j*interval[0], start_pose[1]+j*interval[1], start_pose[2]+j*interval[2]};
            if(!collisionFreeCartesian(robot_config)){
                return false;
            }
        }
    }
    return true;
}

bool SamplingPlanner::collisionFreeCartesian(vector<double> pt){
    double x = pt[0];
    double y = pt[1];
    double z = pt[2];
    double offset = 0.2;

    if(pt[2] <= 0 ){
        return false;
    }

    // Obs 1
    if(x >= OBS1[0] - OBS1[3]/2 - offset && x <= OBS1[0] + OBS1[3]/2 + offset && 
       y >= OBS1[1] - OBS1[4]/2 - offset && y <= OBS1[1] + OBS1[4]/2 + offset && 
       z >= OBS1[2] - OBS1[5]/2 - offset && z <= OBS1[2] + OBS1[5]/2 + offset){
           return false;
       }
    // Obs 2
    if(x >= OBS2[0] - OBS2[3]/2 - offset && x <= OBS2[0] + OBS2[3]/2 + offset && 
       y >= OBS2[1] - OBS2[4]/2 - offset && y <= OBS2[1] + OBS2[4]/2 + offset && 
       z >= OBS2[2] - OBS2[5]/2 - offset && z <= OBS2[2] + OBS2[5]/2 + offset){
           return false;
       }
    // Obs 3
    if(x >= OBS3[0] - OBS3[3]/2 - offset && x <= OBS3[0] + OBS3[3]/2 + offset && 
       y >= OBS3[1] - OBS3[4]/2 - offset && y <= OBS3[1] + OBS3[4]/2 + offset && 
       z >= OBS3[2] - OBS3[5]/2 - offset && z <= OBS3[2] + OBS3[5]/2 + offset){
           return false;
       }
    // Obs 4
    if(x >= OBS4[0] - OBS4[3]/2 - offset && x <= OBS4[0] + OBS4[3]/2 + offset && 
       y >= OBS4[1] - OBS4[4]/2 - offset && y <= OBS4[1] + OBS4[4]/2 + offset && 
       z >= OBS4[2] - OBS4[5]/2 - offset && z <= OBS4[2] + OBS4[5]/2 + offset){
           return false;
       }
    return true;
}


void SamplingPlanner::backTrack(){
    Point6D cur_pt = goal_;
    int idx = getIndex(cur_pt);

    double distance_cost = 0;
    Point6D pre_pt = cur_pt;
    while(!equalPt(cur_pt, start_))
    {
        searchPath.push(cur_pt);

        pre_pt = cur_pt;
        cur_pt = cellInfo[idx].parent;
        distance_cost = distance_cost + distBetweenConfigs(pre_pt, cur_pt);
        idx = getIndex(cur_pt);
    }
    
    cout<<"Distance cost: "<<distance_cost<<endl;
    searchPath.push(start_);
    return;
}

bool SamplingPlanner::equalPt(Point6D A, Point6D B){
    for(int i=0; i<ROBOT_DOF; i++){
        if(abs(A[i] - B[i]) > EQUAL_TOL){
            return false;
        }
    }
    return true;
}

void SamplingPlanner::calcFK(Point6D pt, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr){
    Eigen::MatrixXd robot_pose = Eigen::MatrixXd::Identity(4, 4);
    vector<double> th{0, pt[0], pt[1], 0, pt[2], pt[3], pt[4], pt[5], 0};
    for(int i=0; i<9; i++){
        double thi = th[i] + DH_th[i];
        
        double alphai = DH_alpha[i];
        double ai = DH_a[i];
        double di = DH_d[i];
        Eigen::MatrixXd Ri(4, 4);
        Ri << cos(thi), -sin(thi) * cos(alphai), sin(thi) * sin(alphai), ai * cos(thi),
              sin(thi), cos(thi) * cos(alphai), -cos(thi) * sin(alphai), ai * sin(thi),
              0, sin(alphai), cos(alphai), di,
              0, 0, 0, 1;
        robot_pose = robot_pose * Ri;
       
        ROBOT_POSE(i, 0) = robot_pose(0, 3);
        ROBOT_POSE(i, 1) = robot_pose(1, 3);
        ROBOT_POSE(i, 2) = robot_pose(2, 3);
    }
}
    
bool SamplingPlanner::goalReached(Point6D pt, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr){
    if(distBetweenConfigs(pt, goal_) <= REACH_THRESH && validP2P(pt, goal_, robot_ptr)){
        return true;
    }
    return false;
}

bool SamplingPlanner::reachedPt(Point6D p1, Point6D p2, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr){
    if(distBetweenConfigs(p1, p2) <= REACH_THRESH && validP2P(p1, p2, robot_ptr)){
        return true;
    }
    return false;
}

Point6D SamplingPlanner::reachedPtInList(Point6D pt, 
                                         priority_queue<F_Point6D, vector<F_Point6D>, greater<F_Point6D>> list_pt, 
                                         const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr){
    while(!list_pt.empty()){
        F_Point6D tmp_fpt = list_pt.top();
        list_pt.pop();
        Point6D tmp_pt{tmp_fpt[1], tmp_fpt[2], tmp_fpt[3], tmp_fpt[4], tmp_fpt[5], tmp_fpt[6]};
        if(reachedPt(pt, tmp_pt, robot_ptr)){
            return tmp_pt;
        }
    }
  return pt;
}

void SamplingPlanner::backTrackRRTConnect(Point6D mutual_pt){
    stack<Point6D> tmpPath;
    Point6D cur_pt = mutual_pt;
    int idx = getIndex(cur_pt);
    double distance_cost = 0;
    Point6D pre_pt = cur_pt;
    while(!equalPt(cur_pt, goal_)){
        tmpPath.push(cur_pt);
        cur_pt = backCellInfo[idx].parent;
        idx = getIndex(cur_pt);
    }
    tmpPath.push(goal_);
    
    while(!tmpPath.empty()){
        Point6D tmp_pt = tmpPath.top();
        tmpPath.pop();
        searchPath.push(tmp_pt);
    }

    cur_pt = mutual_pt;
    idx = getIndex(cur_pt);
    while(!equalPt(cur_pt, start_)){
        searchPath.push(cur_pt);
        cur_pt = cellInfo[idx].parent;
        idx = getIndex(cur_pt);
    }
    searchPath.push(start_);
}

Point6D SamplingPlanner::furthestApproachablePt(Point6D start_pt, Point6D end_pt, const std::unique_ptr<robot_interface::RobotInterface>& robot_ptr){
    Point6D pre_pt = start_pt;
    for(int step=1; step<=CONNECTION_RES; step++){
        Point6D tmp_pt = start_pt;
        for(int i=0; i<ROBOT_DOF; i++){
            tmp_pt[i] = start_pt[i]+(end_pt[i]-start_pt[i])/CONNECTION_RES*step;
        }
        if(!validConfig(tmp_pt, robot_ptr)){
            return pre_pt;
        }
        pre_pt = tmp_pt;
    }
    return end_pt;
}


Point6D SamplingPlanner::closestPtInList(Point6D pt, priority_queue<F_Point6D, vector<F_Point6D>, greater<F_Point6D>> list_pt){
    double min_dist = 10000;
    Point6D closest_pt = pt;
    while(!list_pt.empty()){
        F_Point6D tmp_fpt = list_pt.top();
        list_pt.pop();

        Point6D tmp_pt{tmp_fpt[1], tmp_fpt[2], tmp_fpt[3], tmp_fpt[4], tmp_fpt[5], tmp_fpt[6]};
        double dist = distBetweenConfigs(pt, tmp_pt);
        if(dist < min_dist){
            min_dist = dist;
            closest_pt = tmp_pt;
        }
    }
    return closest_pt;
}


