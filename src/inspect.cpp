#include "moveit_recorder/trajectory_video_lookup.h"
#include "moveit_recorder/utils.h"
#include <boost/format.hpp>
#include <boost/program_options.hpp>

int main(int argc, char** argv)
{
  boost::program_options::options_description desc;
  desc.add_options()
    ("save_dir", boost::program_options::value<std::string>(), "Directory to store the recorded bagfile of viewpoints");
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);
  std::string save_dir =utils::get_option(vm, "save_dir", "");
  
  TrajectoryVideoLookup video_lookup_table;
  video_lookup_table.loadFromBag(save_dir);

  TrajectoryVideoLookup::iterator trajectory_it = video_lookup_table.begin();
  for( ; trajectory_it!=video_lookup_table.end(); ++trajectory_it)
  {
    std::string traj_id = trajectory_it->first;
    std::string psname = trajectory_it->second.ps.name;

    std::cout << boost::str(boost::format("traj: %25s scene: %25s") %traj_id %psname) << std::endl;
    TrajectoryVideoLookupEntry::iterator video_it = trajectory_it->second.begin();
    for( ; video_it!=trajectory_it->second.end(); ++video_it )
      std::cout << boost::str(boost::format("%25s | \"%25s\" | \"%25s\"") % video_it->name % video_it->file % video_it->url ) << std::endl;
  }

  return 0;
}
