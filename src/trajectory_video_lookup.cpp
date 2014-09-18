#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/warehouse/planning_scene_storage.h>

#include <algorithm>
#include "moveit_recorder/trajectory_video_lookup.h"

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

void TrajectoryVideoLookupEntry::addVideoFile(const std::string& name, const std::string& file)
{
  TrajectoryVideoLookupEntry::iterator iter = begin();
  for( ;iter!=end();++iter)
  {
    if(name==iter->name)
    {
      iter->file = file;
      return;
    }
  }
  if(iter==end())
  {
    TrajectoryVideoEntry entry;
    entry.name = name;
    entry.file = file;
    videos.push_back(entry);
  }
}

void TrajectoryVideoLookupEntry::addVideoURL(const std::string& name, const std::string& url)
{
  TrajectoryVideoLookupEntry::iterator iter = begin();
  for( ;iter!=end();++iter)
  {
    if(name==iter->name)
    {
      iter->url = url;
      return;
    }
  }
  if(iter==end())
  {
    TrajectoryVideoEntry entry;
    entry.name = name;
    entry.url = url;
    videos.push_back(entry);
  }
}

void TrajectoryVideoLookupEntry::addView(const view_controller_msgs::CameraPlacement& view)
{
  views.push_back(view);
}

TrajectoryVideoLookupEntry::iterator TrajectoryVideoLookupEntry::begin()
{
  return videos.begin();
}

TrajectoryVideoLookupEntry::iterator TrajectoryVideoLookupEntry::end()
{
  return videos.end();
}

TrajectoryVideoLookup::TrajectoryVideoLookup() {}
TrajectoryVideoLookup::~TrajectoryVideoLookup() {}

bool TrajectoryVideoLookup::hasEntry(const std::string& tkey, TrajectoryHashtable::iterator& got)
{
  got = table_.find(tkey);
  if(got!=table_.end())
    return true;
  else
    return false;
}

void TrajectoryVideoLookup::createEntry(const std::string& tkey, TrajectoryHashtable::iterator& got)
{
  TrajectoryVideoLookupEntry new_entry;
  table_.insert( std::pair<std::string, TrajectoryVideoLookupEntry>(tkey, new_entry) );
  got = table_.find(tkey);
}

void TrajectoryVideoLookup::put(const std::string& tkey, moveit_msgs::PlanningScene ps)
{
  TrajectoryHashtable::iterator entry;
  if( !hasEntry(tkey, entry) )
    createEntry(tkey, entry);
  entry->second.ps = ps;
}

void TrajectoryVideoLookup::put(const std::string& tkey, moveit_msgs::MotionPlanRequest mpr )
{
  TrajectoryHashtable::iterator entry;
  if( !hasEntry(tkey, entry) )
    createEntry(tkey, entry);
  entry->second.mpr = mpr;
}

void TrajectoryVideoLookup::put(const std::string& tkey, moveit_msgs::RobotTrajectory rt)
{
  TrajectoryHashtable::iterator entry;
  if( !hasEntry(tkey, entry) )
    createEntry(tkey, entry);
  entry->second.rt = rt;
}

void TrajectoryVideoLookup::put(const std::string& tkey, view_controller_msgs::CameraPlacement view)
{
  TrajectoryHashtable::iterator entry;
  if( !hasEntry(tkey, entry) )
    createEntry(tkey, entry);
  entry->second.addView( view );
}

void TrajectoryVideoLookup::put(const std::string& tkey, TrajectoryVideoEntry vid)
{
  TrajectoryHashtable::iterator entry;
  if( !hasEntry(tkey, entry) )
    createEntry(tkey, entry);
  entry->second.addVideoFile(vid.name, vid.file);
  entry->second.addVideoURL(vid.name, vid.url);
}

void TrajectoryVideoLookup::putVideoFile(const std::string& tkey, const std::string& vkey, const std::string& file)
{
  TrajectoryHashtable::iterator entry;
  if( !hasEntry(tkey, entry) )
    createEntry(tkey, entry);
  entry->second.addVideoFile(vkey, file);
}

void TrajectoryVideoLookup::putVideoURL(const std::string& tkey, const std::string& vkey, const std::string& url)
{
  TrajectoryHashtable::iterator entry;
  if( !hasEntry(tkey, entry) )
    createEntry(tkey, entry);
  entry->second.addVideoURL(vkey, url);
}

TrajectoryVideoLookupEntry TrajectoryVideoLookup::get(const std::string& tkey)
{
  return table_[tkey];
}
moveit_msgs::PlanningScene TrajectoryVideoLookup::getPlanningScene(const std::string& tkey)
{
  return table_[tkey].ps;
}
moveit_msgs::MotionPlanRequest TrajectoryVideoLookup::getMotionPlanRequest(const std::string& tkey)
{
  return table_[tkey].mpr;
}
moveit_msgs::RobotTrajectory TrajectoryVideoLookup::getRobotTrajectory(const std::string& tkey)
{
  return table_[tkey].rt;
}
TrajectoryVideoLookupEntry::VideoList TrajectoryVideoLookup::getVideos(const std::string& tkey)
{
  return table_[tkey].videos;
}
TrajectoryVideoEntry TrajectoryVideoLookup::getNamedVideo(const std::string& tkey, const std::string& vkey)
{
  TrajectoryVideoLookupEntry::iterator iter = table_[tkey].begin();
  for( ;iter!=table_[tkey].end();++iter)
  {
    if(iter->name == vkey)
      return *iter;
  }
  return *iter;
}

bool TrajectoryVideoLookup::hasVideoFile(const std::string& tkey, const std::string& vkey, TrajectoryVideoLookupEntry::iterator& got)
{
  TrajectoryVideoLookupEntry::iterator iter = table_[tkey].begin();
  for(; iter!=table_[tkey].end();++iter)
  {
    if(iter->name == vkey)
      return true;
  }
  return false;
}

std::string TrajectoryVideoLookup::getVideoFile(const std::string& tkey, const std::string& vkey)
{
  TrajectoryVideoLookupEntry::iterator iter = table_[tkey].begin();
  for( ;iter!=table_[tkey].end();++iter)
  {
    if(iter->name == vkey)
      return iter->file;
  }
  return "";
}

void TrajectoryVideoLookup::parseBagFile(rosbag::Bag& bag)
{
  // get list of bag topics
  rosbag::View view_all(bag);
  std::vector<std::string> topics;
  std::vector<const rosbag::ConnectionInfo *> connection_infos = view_all.getConnections();
  BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos)
    topics.push_back(info->topic);
  rosbag::View view_topics(bag, rosbag::TopicQuery(topics));

  // iterate over all messages in all topics
  for( rosbag::View::iterator bagview_it=view_topics.begin(); bagview_it!=view_topics.end(); ++bagview_it)
  {
    // decompose topic into identifiers for trajectory video
    boost::filesystem::path topic_path(bagview_it->getTopic());
    std::vector<std::string> path_decomp;
    boost::filesystem::path::iterator path_it;
    for( path_it = topic_path.begin(); path_it!=topic_path.end(); ++path_it)
    {
      std::string path_string = path_it->c_str();
      if(path_string!="/") //skip root
        path_decomp.push_back( path_string );
    }

    // check if the path decomposition is a valid entry for the table
    if( path_decomp.size() == 2 ) // is one of the message types
    {
      if(path_decomp[1]=="ps")
      {
        moveit_msgs::PlanningScene::Ptr psmsg = bagview_it->instantiate<moveit_msgs::PlanningScene>();
        if(psmsg!=NULL)
        {
          ROS_INFO("Loaded planning scene for trajectory id \"%s\"", path_decomp[0].c_str());
          put( path_decomp[0], *psmsg );
        }
        else
          ROS_WARN("No saved planning scene for trajectory id \'%s\'",path_decomp[0].c_str());
        continue;
      }
      if(path_decomp[1]=="mpr")
      {
        moveit_msgs::MotionPlanRequest::Ptr mprmsg = bagview_it->instantiate<moveit_msgs::MotionPlanRequest>();
        if(mprmsg!=NULL)
        {
          ROS_INFO("Loaded motion plan request for trajectory id \"%s\"", path_decomp[0].c_str());
          put( path_decomp[0], *mprmsg );
        }
        else
          ROS_WARN("No saved planning request for trajectory id \'%s\'",path_decomp[0].c_str());
        continue;
      }
      if(path_decomp[1]=="rt")
      {
        moveit_msgs::RobotTrajectory::Ptr rtmsg = bagview_it->instantiate<moveit_msgs::RobotTrajectory>();
        if(rtmsg!=NULL)
        {
          ROS_INFO("Loaded robot trajectory for trajectory id \"%s\"", path_decomp[0].c_str());
          put( path_decomp[0], *rtmsg );
        }
        else
          ROS_WARN("No saved trajectory for trajectory id \'%s\'",path_decomp[0].c_str());
        continue;
      }
      if(path_decomp[1]=="views")
      {
        view_controller_msgs::CameraPlacement::Ptr viewmsg = bagview_it->instantiate<view_controller_msgs::CameraPlacement>();
        if(viewmsg!=NULL)
        {
          ROS_INFO("Loaded view for trajectory id \"%s\"", path_decomp[0].c_str());
          put( path_decomp[0], *viewmsg );
        }
        else
          ROS_WARN("No saved view for trajectory id \'%s\'",path_decomp[0].c_str());
        continue;
      }
    }
    else if(path_decomp.size() == 3 ) // is a video file -or- url
    {
      std_msgs::String::Ptr strmsg = bagview_it->instantiate<std_msgs::String>();
      if(strmsg!=NULL)
      {
        if(path_decomp[1]=="vid")
        {
          ROS_INFO("Loaded video \"%s\" for trajectory id \"%s\"", path_decomp[2].c_str(), path_decomp[0].c_str());
          putVideoFile( path_decomp[0], path_decomp[2], strmsg->data);
        }
        if(path_decomp[1]=="url")
        {
          ROS_INFO("Loaded url \"%s\" for trajectory id \"%s\"", path_decomp[2].c_str(), path_decomp[0].c_str());
          putVideoURL( path_decomp[0], path_decomp[2], strmsg->data);
        }
      }
    }
    else
      continue;
  }
}

void TrajectoryVideoLookup::writeBagFile(rosbag::Bag& bag)
{
  // iterate over all entries
  TrajectoryHashtable::iterator entry = table_.begin();
  size_t trajectory_count=0;
  for( ; entry!=table_.end(); ++entry)
  {
    // /<traj>/<type>/<name> 
    boost::filesystem::path topic("/");
    topic = topic / entry->first;
    bag.write<moveit_msgs::PlanningScene>( (topic/"ps").string() , ros::Time::now(), entry->second.ps);
    bag.write<moveit_msgs::MotionPlanRequest>( (topic/"mpr").string() , ros::Time::now(), entry->second.mpr);
    bag.write<moveit_msgs::RobotTrajectory>( (topic/"rt").string() , ros::Time::now(), entry->second.rt);
    
    size_t video_count = 0;
    TrajectoryVideoLookupEntry::iterator video_entry = entry->second.begin();
    for( ; video_entry!=entry->second.end(); ++video_entry)
    {
      std_msgs::String filemsg, urlmsg;
      filemsg.data = video_entry->file;
      urlmsg.data = video_entry->url;
      bag.write<std_msgs::String>( (topic/"vid"/video_entry->name).string(), ros::Time::now(), filemsg);
      bag.write<std_msgs::String>( (topic/"url"/video_entry->name).string(), ros::Time::now(), urlmsg);
      
      video_count++;
    }
    ROS_INFO("Saved %d videos", (int) video_count );

    size_t view_count = 0;
    std::vector<view_controller_msgs::CameraPlacement>::iterator view_it = entry->second.views.begin();
    for(;view_it!=entry->second.views.end();++view_it)
    {
      bag.write<view_controller_msgs::CameraPlacement>( (topic/"views").string(), ros::Time::now(), *view_it);
      view_count++;
    }
    ROS_INFO("Saved %d views", (int) view_count );
    trajectory_count++;
  }
  ROS_INFO("Saved %d trajectories", (int) trajectory_count );
}
void TrajectoryVideoLookup::initializeFromWarehouse(const std::string& host, const size_t port)
{
  moveit_warehouse::PlanningSceneStorage pss(host, port);
  ROS_INFO("Connected to Warehouse DB at host (%s) and port (%d)", host.c_str(), (int)port);
  
  // ask the warehouse for the scenes
  std::vector<std::string> ps_names;
  pss.getPlanningSceneNames( ps_names );
  ROS_INFO("%d available scenes to display", (int)ps_names.size());
  std::vector<std::string>::iterator scene_name = ps_names.begin();
  for(; scene_name!=ps_names.end(); ++scene_name)
  {
    ROS_INFO("Retrieving scene %s", scene_name->c_str());
    moveit_warehouse::PlanningSceneWithMetadata pswm;
    pss.getPlanningScene(pswm, *scene_name);
    moveit_msgs::PlanningScene ps_msg = static_cast<const moveit_msgs::PlanningScene&>(*pswm);
    
    // ask qarehosue for the queries
    std::vector<std::string> pq_names;
    pss.getPlanningQueriesNames( pq_names, *scene_name);
    ROS_INFO("%d available queries to display", (int)pq_names.size());

    // iterate over the queries
    std::vector<std::string>::iterator query_name = pq_names.begin();
    for(; query_name!=pq_names.end(); ++query_name)
    {
      ROS_INFO("Retrieving query %s", query_name->c_str());
      moveit_warehouse::MotionPlanRequestWithMetadata mprwm;
      pss.getPlanningQuery(mprwm, *scene_name, *query_name);
      moveit_msgs::MotionPlanRequest mpr_msg = static_cast<const moveit_msgs::MotionPlanRequest&>(*mprwm);

      // ask warehouse for stored trajectories
      std::vector<moveit_warehouse::RobotTrajectoryWithMetadata> planning_results;
      pss.getPlanningResults(planning_results, *scene_name, *query_name);
      ROS_INFO("Loaded %d trajectories", (int)planning_results.size());

      // animate each trajectory
      std::vector<moveit_warehouse::RobotTrajectoryWithMetadata>::iterator traj_w_mdata = planning_results.begin();
      for(; traj_w_mdata!=planning_results.end(); ++traj_w_mdata)
      {
        moveit_msgs::RobotTrajectory rt_msg;
        rt_msg = static_cast<const moveit_msgs::RobotTrajectory&>(**traj_w_mdata);
        //date and time based filename
        boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
        std::string time_as_string = boost::posix_time::to_simple_string(now);

        // create trajectory ID
        std::string trajectory_id = time_as_string;
        std::replace(trajectory_id.begin(), trajectory_id.end(), '/','_');
        std::replace(trajectory_id.begin(), trajectory_id.end(), '-','_');
        std::replace(trajectory_id.begin(), trajectory_id.end(), ' ','_');
        std::replace(trajectory_id.begin(), trajectory_id.end(), ':','_');

        // save into lookup
        put( trajectory_id, ps_msg );
        put( trajectory_id, mpr_msg );
        put( trajectory_id, rt_msg );

        sleep(1);
      }
    }
  }
}

void TrajectoryVideoLookup::loadFromBag(const std::string& bag_filepath)
{
  rosbag::Bag bag;
  if( openBag( bag_filepath, rosbag::bagmode::Read, bag ) )
  {
    parseBagFile( bag );
    bag.close();
  }
}
void TrajectoryVideoLookup::saveToBag(const std::string& bag_filepath)
{
  rosbag::Bag bag;
  if( openBag( bag_filepath, rosbag::bagmode::Write, bag ) )
  {
    writeBagFile(bag);
    bag.close();
  }
}

bool TrajectoryVideoLookup::openBag(const std::string& bagpath, const rosbag::bagmode::BagMode bagmode, rosbag::Bag& bag)
{
  boost::filesystem::path path(bagpath);
  if(boost::filesystem::is_directory(path))
  {
    // check for default bag file name "video_lookup.bag"
    path = path / "video_lookup.bag";
    bag.open( path.string(), bagmode );
    return true;
  }
  else
  {
    // check if a bag file
    if( path.extension().c_str() == ".bag" )
    {
      // is it a bag file?
      if( boost::filesystem::exists( path ) || bagmode==rosbag::bagmode::Write )
      {
        // proceed to read
        bag.open(path.string(), bagmode);
        return true;
      }
      else
      {
        // no file to read
        ROS_INFO("No such bag file exists.");
      }
    }
    else
    {
      // can't read this file
      ROS_INFO("Provided file is not a bag: %s", path.extension().c_str() );
    }
  }
  return false;
}
