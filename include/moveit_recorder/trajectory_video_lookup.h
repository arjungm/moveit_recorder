#ifndef TRAJ_VID_LOOKUP
#define TRAJ_VID_LOOKUP

#include <string>
#include <map>

#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <std_msgs/String.h>

struct TrajectoryVideoEntry
{
  std::string file;
  std::string url;
};

struct TrajectoryVideoLookupEntry
{
  typedef std::map<std::string, TrajectoryVideoEntry> VideoHashtable;
  moveit_msgs::PlanningScene ps;
  moveit_msgs::MotionPlanRequest mpr;
  moveit_msgs::RobotTrajectory rt;
  VideoHashtable videos;
};

class TrajectoryVideoLookup
{
  typedef std::map<std::string, TrajectoryVideoLookupEntry> TrajectoryHashtable;
  public:
    TrajectoryVideoLookup();
    ~TrajectoryVideoLookup();
    
    bool hasEntry(const std::string& tkey, TrajectoryHashtable::iterator& got);
    void createEntry(const std::string& tkey, TrajectoryHashtable::iterator& got);
    void put(const std::string& tkey, moveit_msgs::PlanningScene ps);
    void put(const std::string& tkey, moveit_msgs::MotionPlanRequest mpr );
    void put(const std::string& tkey, moveit_msgs::RobotTrajectory rt);
    void put(const std::string& tkey, const std::string& vkey, TrajectoryVideoEntry vid);
    void putVideoFile(const std::string& tkey, const std::string& vkey, std::string file);
    void putVideoURL(const std::string& tkey, const std::string& vkey, std::string url);

    TrajectoryVideoLookupEntry get(const std::string& tkey);
    moveit_msgs::PlanningScene getPlanningScene(const std::string& tkey);
    moveit_msgs::MotionPlanRequest getMotionPlanRequest(const std::string& tkey);
    moveit_msgs::RobotTrajectory getRobotTrajectory(const std::string& tkey);
    std::map<std::string, TrajectoryVideoEntry> getVideos(const std::string& tkey);
    TrajectoryVideoEntry getNamedVideo(const std::string& tkey, const std::string& vkey);

    void loadFromBag(const std::string& bag_filepath);
    void saveToBag(const std::string& bag_filepath);
    void parseBagFile(rosbag::Bag& bag);
    void writeBagFile(rosbag::Bag& bag);
    bool openBag(const std::string& bagpath, const rosbag::bagmode::BagMode bagmode, rosbag::Bag& bag);
  
  protected:
    
  private:
    TrajectoryHashtable table_;
};

#endif
