#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/date_time.hpp>
#include <boost/shared_ptr.hpp>
#include "gridfastslam/gridslamprocessor.h"

class PointsCloud
{
public:
    std::vector<cv::Point2f> pts;
    std::vector<double> dists;
    std::vector<double> angles;
    long long int ts;
};

class RobotState
{
public:
    cv::Point3f pos;
    long long int ts;
    double line_spd;
    double rot_spd;
};

std::ostream &operator << (std::ostream &os, const RobotState &rb)
{
    os << "state: " << rb.pos << ", " << cv::Vec2d(rb.line_spd, rb.rot_spd) << ", ts: " << rb.ts;
    return os;
}

boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
class LidarPCloudTest
{
public:
    LidarPCloudTest()
    {
        lidar_log = "../data/lidar.log";
        odom_log = "../data/odom.log";

        // iss.imbue(std::locale(iss.getloc(), new boost::posix_time::time_input_facet("%Y%m%d.%H%M%S%f")));
    }
    ~LidarPCloudTest()
    {
        ifs.close();
    }

    std::string lidar_log, odom_log;
    std::ifstream ifs;
    
    // std::stringstream iss;

    bool ConvertStampFromString(std::string string_format, long long int &ts)
    {
        if (string_format.length() == 13)
            ts = strtoll(string_format.c_str(), NULL, 0);
        else if (string_format.length() == 19)
            ts = strtoll(string_format.c_str(), 0, 0) / ((int)1e6);
        return true;
    }

    bool ReadLidarPoints(PointsCloud &pcs)
    {
        double dists[720] = {0};
        double angles[720] = {0};
        for (int i = 0; i < 720; i++)
        {
            angles[i] = (i - 360)/2.0 / 180 * M_PI;
        }
        pcs.pts.clear();
        pcs.dists.clear();
        pcs.angles.clear();
        // pcs.ts = boost::posix_time::ptime(boost::posix_time::not_a_date_time);
        // pcs.ts = 0;
        // epoch
       
        std::string line;
        if (!ifs.is_open()) ifs.open(lidar_log.c_str(), std::ios_base::in);
        while (!ifs.eof())
        {
            std::getline(ifs, line);
            if (line.length() == 0)
                continue;
            
            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of(":,;"));
            
            ConvertStampFromString(strs[0], pcs.ts);
            
            int index = 0;
            for (size_t i = 2; i < strs.size() - 1; i+=2)
            {
                // float nangle = (float)(int(strtof(strs[i].c_str(), 0)) >> 1)/64.0f;
                // nangle = ChangeLidarAngle(nangle);
                // float distance = (float)strtof(strs[i+1].c_str(), 0)/4.0f/10;  // cm
                // double fangle = nangle*M_PI/180;
                double fangle = strtof(strs[i].c_str(), 0);
                float tmp = fangle;
                // turn laser pose align to odom coordinate
                fangle = -(fangle - M_PI_2);
                fangle = fmod( fangle, 2*M_PI );
                if (fangle > M_PI)
                    fangle -= 2*M_PI;              // unit rad (-pi, pi]
                std::cout << "fangle = " << fangle << ", tmp = " << tmp << std::endl;

                float distance = strtof(strs[i+1].c_str(), 0);// unit m
                cv::Point2f point;
                point.x = cos(fangle)*distance;   // X向前为正
                point.y = sin(fangle)*distance;  // Y向左为正

                // point = changelidar2odom(point*100, M_PI, 15, 0);
                // point *= 100;
                // pcs.pts.push_back(point);
                
                // pcs.dists.push_back(distance);
                // pcs.angles.push_back(fangle);
                index = (fangle / M_PI * 180 + 180) * 2;
                dists[index] = distance;
                angles[index] = fangle;
                // int index = (fangle / M_PI * 180 + 180) * 2;
                // dists[index] = distance;
                // angles[index] = fangle /* /M_PI*180 */;
            }
            pcs.dists.assign(dists, dists+720);
            pcs.angles.assign(angles, angles+720);

            return true;
        }
        return false;
    }

    void normalizeAngle(float &rad)
    {
        rad = fmod(rad, 2*M_PI);
        if  (rad > M_PI)
            rad -= 2 * M_PI;
    }

    cv::Point3f motion(RobotState state, float dt)
    {
        normalizeAngle(state.pos.z);
        state.pos.x += state.line_spd * cos(state.pos.z) * dt;
        state.pos.y += state.line_spd * sin(state.pos.z) * dt;
        return state.pos;
    }

    std::vector<RobotState> robotstates; 
    void ReadRobotSpd()
    {
        std::ifstream ifs;
        if (!ifs.is_open()) ifs.open(odom_log.c_str(), std::ios_base::in);
        std::string line;

        RobotState last_state;
        while (!ifs.eof())
        {
            std::getline(ifs, line);
            if (line.length() == 0)
                continue;
            
            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of(","));

            float orientation = strtof(strs[1].c_str(), 0); // rad
            float line_spd = strtof(strs[2].c_str(), 0) * 10; // line spd cm/s
            RobotState st;
            st.pos.z = orientation;
            st.line_spd = line_spd / 100.0; // cm/s ==> m/s
            ConvertStampFromString(strs[0], st.ts);

            if (robotstates.size() == 0)
            {
                st.pos = cv::Point3f(0, 0, st.pos.z);
                robotstates.push_back(st);
                continue;
            }
            else
                last_state = robotstates.back();
            
            st.pos = motion(last_state, (st.ts - last_state.ts)/1000.0f);
            st.pos.z = orientation;
            // st.line_spd = line_spd / 100.0;
            st.rot_spd = (-last_state.pos.z + st.pos.z) / ((-last_state.ts + st.ts)/1000.0);
            normalizeAngle(st.pos.z);
            std::cout << st << ", orien: " << orientation << ", dt: " << (st.ts - last_state.ts)/1000.0f << std::endl;
            robotstates.push_back(st);
        }
    }

    int findClosestStateByTS(long long int ts, int start, int end)
    {
        if (end - start <= 1)
        {
            return abs(robotstates[end-1].ts - ts) <= abs(robotstates[end].ts - ts) ? end -1 : end;
        }

        int mid = (start + end) / 2;
        if (robotstates[mid].ts == ts)
            return mid;
        if (robotstates[mid].ts < ts && ts < robotstates[mid+1].ts)
            return abs(robotstates[mid].ts - ts) <= abs(robotstates[mid+1].ts - ts) ? mid : mid + 1;
        else if (robotstates[mid].ts > ts)
            return findClosestStateByTS(ts, start, mid);
        else if (robotstates[mid].ts < ts)
            return findClosestStateByTS(ts, mid + 1, end);
    }
};

int main(int, char**) {
    GMapping::OdometrySensor odom_sensor("ODOM", false);
    // GMapping::RangeSensor lidar_sensor("FLASER", 720, 0.5, GMapping::OrientedPoint(15, 0, 0));
    double angles[720] = {0};
    for (int i = 0; i < 720; i++)
    {
        angles[i] = (i - 360)/2.0 / 180 * M_PI;
    }
    GMapping::RangeSensor lidar_sensor("FLASER", 720, angles, GMapping::OrientedPoint(0.15, 0, 0));

    GMapping::OdometryReading odom_reader(&odom_sensor, 0);
    GMapping::RangeReading lidar_reader(&lidar_sensor, 0);

    LidarPCloudTest lpt;
    lpt.ReadRobotSpd();
    GMapping::GridSlamProcessor gsp;
    gsp.init(30, -20, -20, 20, 20, 0.05);
    // gsp.setlaserPose(GMapping::OrientedPoint(20, 0, 0));
    // gsp.setlaserMaxRange(10);
    gsp.setMatchingParameters(15, 15, 0.05, 1, 0.05, 0.05, 5/* , 0.075 *//* , 3.0, 0 */);
    gsp.setMotionModelParameters(0.1, 0.05, 0.1, 0.2);
    // gsp.setUpdateDistances(0.1, 0.2, 0.5);
    gsp.setUpdateDistances(0.1, 0.1, 0.5);
    gsp.setminimumScore(0.0004);
    // gsp.setgenerateMap(true); // alloc memory for free cell between center and hit point
    gsp.setllsamplerange(0.01);
    gsp.setllsamplestep(0.01);
    gsp.setlasamplerange(0.005);
    gsp.setlasamplestep(0.005);

    // lidar_sensor.updateBeamsLookup();
    // if random seeds != 0
    // GMapping::sampleGaussian(1, seeds);

    PointsCloud pcs;
    // boost::posix_time::ptime last_ts;
    long long int last_ts = -1;

    GMapping::SensorMap smap;
    smap.insert(std::make_pair(lidar_sensor.getName(), &lidar_sensor));
    smap.insert(std::make_pair(odom_sensor.getName(), &odom_sensor));
    gsp.setSensorMap(smap);

    std::vector<std::pair<PointsCloud, RobotState> > prs;
    cv::Mat img(1000, 1000, CV_8UC3, cv::Scalar(0));
    cv::namedWindow("img", CV_WINDOW_NORMAL);

    while (lpt.ReadLidarPoints(pcs))
    {
        if (last_ts == -1)
            last_ts = pcs.ts;
        // RobotState st = lpt.GetRobotStateByTS(pcs.ts, min_index);
        int local_index = lpt.findClosestStateByTS(pcs.ts, 0, lpt.robotstates.size()-1);
        RobotState st = lpt.robotstates[local_index];
        
        GMapping::RangeReading *reading = new GMapping::RangeReading(pcs.dists.size(), pcs.dists.data(), 
                    pcs.angles.data(), &lidar_sensor, pcs.ts);
        (*reading).setPose(GMapping::OrientedPoint(st.pos.x, st.pos.y, st.pos.z));
        std::cout << "time: " << (long long int)reading->getTime() << std::endl;
        std::cout << (*reading).getPose() << std::endl;
        gsp.processScan(*reading, 30);
        last_ts = pcs.ts;

        int max_index = gsp.getBestParticleIndex();
        GMapping::GridSlamProcessor::Particle bestP = gsp.getParticles()[max_index];
        std::cout << "max it node accweight: " << bestP.node->accWeight << ", weight = " << bestP.weight << ", sum weight = " << bestP.weightSum << std::endl;
        std::cout << bestP.pose << " ====> " << reading->getPose() << std::endl;
        for (double i =0;i < bestP.map.getMapSizeX(); i++)
        {
            for (double j = 0; j < bestP.map.getMapSizeY(); j++)
            {
                GMapping::IntPoint p(i, j);
                double occ = bestP.map.cell(p);
                // std::cout << "occ = " << occ << std::endl;
                if (occ >= 0.5)
                    cv::circle(img, cv::Point(j, i), 3, cv::Scalar(0, 255, 0));
            }
        }

        cv::imshow("img", img);
        cv::waitKey(0);
        img = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(0));
    }

    return 0;
}

/* LidarPCloudTest lpt2;
    std::vector<std::pair<PointsCloud, RobotState> > prs2;
    PointsCloud pcs2;
    lpt2.ReadRobotSpd();
    while (lpt2.ReadLidarPoints(pcs2))
    {
        int local_index = lpt2.findClosestStateByTS(pcs2.ts, 0, lpt2.robotstates.size()-1);
        RobotState st = lpt2.robotstates[local_index];
        prs2.push_back(std::make_pair(pcs2, st));
    }

    
    std::list<std::pair<PointsCloud, RobotState> > saved2;
    for (size_t i = 0; i < prs2.size(); i++)
    {
        saved2.push_back(prs2[i]);
        if (saved2.size() > 15)
            saved2.pop_front();
        lpt2.CombinePts(saved2);
    }
    return 0; */

    /* 
    RobotState GetRobotStateByTS(long long int ts, int &index)
    {
        int min_tv = INT_MAX;
        RobotState min_ts;
        for (size_t i = 0; i < robotstates.size(); i++)
        {
            double stamp = fabs(robotstates[i].ts - ts);
            if (stamp < min_tv)
            {
                min_ts = robotstates[i];
                min_tv = fabs(robotstates[i].ts - ts);
                index = i;
            }
        }
        return min_ts;
    }
    float ChangeLidarAngle(float angle) 
    {
        float change_angle =  angle;
        if (1) {
            change_angle += 270.0;
            if (change_angle > 360.0) change_angle -= 360.0;
        }
        
        return change_angle;
    } */
    /* cv::Point2f changelidar2odom(cv::Point2f point, float rotate_point_rad = M_PI, float dist_x = 0, float dist_y = 0)
    {
        cv::Point ret_point;
        ret_point.x = point.x * cos(rotate_point_rad) - point.y * sin(rotate_point_rad) + dist_x;
        ret_point.y = point.x * sin(rotate_point_rad) + point.y * cos(rotate_point_rad) + dist_y;
        // LOG_INFO << "before change point to odometer: " << point << ", dist: " << dist_x << ", ret: " << ret_point << ", change: " << change_point;
        return ret_point;
    } 
    bool ConvertPtimeFromString(std::string string_format, boost::posix_time::ptime &tm)
    {
        // strs[0] = "10/06/2020 04:12:21";
        // strs[0] = "20210106_112609_.539032";//%F or %f format must be .123456, not 123456, %s for 02.123456 same as %S%F(%S%f)
        // iss.imbue(std::locale(iss.getloc(), new boost::posix_time::time_input_facet("%m/%d/%Y %H:%M:%S")));
        long long int ts = 0;
        if (string_format.length() == 13) // ms
        {
            ts = strtoll(string_format.c_str(), NULL, 0);
        }
        else if (string_format.length() == 19)
        {
            ts = strtoll(string_format.c_str(), NULL, 0);
            ts = ts / 1000000;
        }
        else {
            std::replace(string_format.begin(), string_format.end(), '_', '.');// must change delim '_' to '.' for %f
            iss.clear();
            iss << string_format;
            iss >> tm;
            return tm.is_not_a_date_time() ? false : true;
        }

        tm = boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1), boost::posix_time::milliseconds(ts));
        return true;
    }
    */
    // void CombinePts(std::list<std::pair<PointsCloud, RobotState> > saved)
    // {

    //     cv::Mat img(2000, 2000, CV_8UC3, cv::Scalar::all(0));
    //     cv::namedWindow("img", CV_WINDOW_NORMAL);
    //     // std::vector<cv::Point2f> all_pts;
    //     // std::cout << "size ====================== " << saved.size() << std::endl;
    //     RobotState coord = saved.back().second;
    //     while (!saved.empty())
    //     {
    //         RobotState st = saved.front().second;
    //         float delta_theta = -(coord.pos.z - st.pos.z);
    //         // if (delta_theta > 180)
    //         //     delta_theta = delta_theta - 360;
    //         // else if (delta_theta < -180)
    //         //     delta_theta = delta_theta + 360;
    //         // delta_theta = delta_theta / 180 * M_PI;
            
    //         float delta_x = coord.pos.x - st.pos.x;
    //         float delta_y = coord.pos.y - st.pos.y;
    //         // std::cout << "source st: " << st << ", end st: " << coord << std::endl;
    //         // std::cout << "delta theta: " << delta_theta << std::endl;
    //         // std::cout << "dleta x&y: " << delta_x << ", "<< delta_y << std::endl;

    //         for (size_t i = 0; i < saved.front().first.pts.size(); i++)
    //         {
    //             float s_arr[3] = {saved.front().first.pts[i].x, saved.front().first.pts[i].y, 1};
    //             // cv::circle(img, cv::Point2f(s_arr[0], s_arr[1]) + cv::Point2f(1000, 1000), 2, cv::Scalar(0, 255, 0));
                
    //             float rt_arr[9] = {cosf(delta_theta), sinf(delta_theta), delta_x, -sinf(delta_theta), cosf(delta_theta), delta_y, 0, 0, 1};
    //             cv::Mat rt_mat(3, 3, CV_32FC1, rt_arr);
    //             cv::Mat res = cv::Mat(1, 3, CV_32FC1, s_arr) * rt_mat.t().inv();

    //             // float rt_arr2[9] = {cosf(-delta_theta), sinf(-delta_theta), 0/* sqrtf(delta_x*delta_x)+(delta_y*delta_y) */, -sinf(-delta_theta), cosf(-delta_theta),0,0,0,1};
    //             // cv::Mat rt_mat2(3, 3, CV_32FC1, rt_arr2);
    //             // cv::Mat res2 = cv::Mat(1, 3, CV_32FC1, s_arr) * rt_mat2.t();

    //             cv::Point2f pos;
    //             pos.x = res.at<float>(0,0);
    //             pos.y = res.at<float>(0,1);
    //             // std::cout << rt_mat.t().inv() << std::endl;
    //             // std::cout << "rt point " << i << ", src_p: "<<cv::Point2f(s_arr[0], s_arr[1]) << ", dst_p: " << pos << std::endl;
    //             // if (i < saved.back().first.pts.size())
    //             //     std::cout << "rp curet: " << saved.back().first.pts[i] << std::endl;
    //             cv::circle(img, pos*100 + cv::Point2f(1000, 1000), 2, cv::Scalar(255, 255, 255));
    //         }

    //         for (size_t i = 0; i < saved.back().first.pts.size(); i++)
    //         {
    //             cv::circle(img, saved.back().first.pts[i]*100 + cv::Point2f(1000, 1000), 2, cv::Scalar(255, 0, 0));
    //         }
    //         // float cosQ = cosf(delta_theta);
    //         // float sinQ = sinf(delta_theta);
    //         // for (size_t i = 0; i < saved.front().first.pts.size(); i++)
    //         // {
    //         //     cv::Point2f pos;
    //         //     pos.x = saved.front().first.pts[i].x * cosQ + saved.front().first.pts[i].y * sinQ + delta_x/* sqrtf(delta_x * delta_x + delta_y * delta_y) */;
    //         //     pos.y = saved.front().first.pts[i].x * -sinQ + saved.front().first.pts[i].y * cosQ + delta_y;
    //         //     // all_pts.push_back(pos);
    //         //     cv::circle(img, pos + cv::Point2f(1000, 1000), 2, cv::Scalar(255, 255, 255));
    //         // }

    //         saved.pop_front();
    //     }
    //     cv::imshow("img", img);
    //     cv::waitKey(0);
    // }