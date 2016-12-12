#include <iostream>

#include "node_options.h"
#include "node.h"

#include "MapPoint2D.h"

//#include "orgdata.h"


DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");


using namespace cartographer_ros;




int main(int argc, char **argv) 
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  cout<<FLAGS_configuration_directory.size()<<endl;
  
   CHECK(!FLAGS_configuration_directory.empty())
   << "-configuration_directory is missing.";
 CHECK(!FLAGS_configuration_basename.empty())
   << "-configuration_basename is missing.";
  
  auto file_resolver =  carto::common::make_unique<carto::common::ConfigurationFileResolver>( std::vector<string>{FLAGS_configuration_directory});
  
  const string code = file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);

  carto::common::LuaParameterDictionary lua_parameter_dictionary( code, std::move(file_resolver), nullptr);
  
  Node node(CreateNodeOptions(&lua_parameter_dictionary));
  
  node.Initialize();
  
  node.SpinForever();
  
  
  My::map2d::MapPoints mps;
  mps.InPutDataFast("/mnt/hgfs/0.TEATDATA/GreenValley/test4/reall.odm", "/mnt/hgfs/0.TEATDATA/GreenValley/test4/da.flam");
  
  for(;;)
  {
		
  }
  cout<<code<<endl;
    return 0;
}
