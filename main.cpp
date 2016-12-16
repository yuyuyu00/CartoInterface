#include <iostream>

#include "node_options.h"
#include "node.h"

#include "MapPoint2D.h"
#include "MapPoint3D.h"

#include <sstream>
#include <string>
//#include "orgdata.h"

#include <chrono>
#include <thread>


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
	
	cout << FLAGS_configuration_directory.size() << endl;
	
	CHECK(!FLAGS_configuration_directory.empty())
	<< "-configuration_directory is missing.";
	CHECK(!FLAGS_configuration_basename.empty())
	<< "-configuration_basename is missing.";
	
	auto file_resolver = carto::common::make_unique<carto::common::ConfigurationFileResolver>(std::vector<string>{FLAGS_configuration_directory});
	
	const string code = file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
	
	carto::common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));
	
	Node node(CreateNodeOptions(&lua_parameter_dictionary));
	
	node.Initialize();
	
	node.SpinForever();
	
	#if 0
	My::map2d::MapPoints mps;
	mps.InPutDataFast("/mnt/hgfs/0.TEATDATA/GreenValley/test4/reall.odm", "/mnt/hgfs/0.TEATDATA/GreenValley/test4/da.flam");
	
	
	for (int i = 0; i < mps.mps.size() - 1; i++)
	{
	node.HandleLaser(*mps.mps[i]);
	cout << i << " " << mps.mps.size() << endl;
	sleep(1);
}
#else
#ifdef WIN32
string datapath("D:/work/program/Cartographer/cartographer/test4/");
#else
string datapath("/mnt/hgfs/E/data/test4/");
#endif // WIN32

//path
//????

vector<string> str;
for (int i = 0; i < 2496; i++)
{
	stringstream ss;
	ss << i;
	string path = datapath + string("sweep") + ss.str() + string(".txt");
	str.push_back(path);
	cout << str[i] << endl;
}

//init read
My::map3d::MapPoint3D mp;
My::IMUDatas imus;
double imutime, lasertime;
int imuindex, laserindex;
imuindex = laserindex = 0;

bool rdokimu = imus.ReadData((datapath + "rep.txt").c_str());
bool rdoklaser = mp.ReadXYZ(str[laserindex].c_str(), 1);
if (!rdokimu || !rdoklaser)
{
	cerr << "read file error" << endl;
	return -1;
}

lasertime = mp.m_tm;
imutime = imus.m_dat[imuindex]->m_tm;
double starttime = imutime;
for (int i = 0;; i++)
{
	if (lasertime > imutime)
	{//??imu
		node.HandleIMU(*(imus.m_dat[imuindex]));
		if (imuindex >= imus.m_dat.size() - 1)
			break;
		imutime = imus.m_dat[++imuindex]->m_tm;
		
	}
	else
	{//????
		node.HandleLaser(mp);
		if (!mp.ReadXYZ(str[++laserindex].c_str(), 1))
			break;
		lasertime = mp.m_tm;
	}
	
	cout << (lasertime > imutime ? imutime : lasertime) - starttime << endl;
	
}




#endif




for (;;)
{
	std::this_thread::sleep_for(std::chrono::nanoseconds(100));
	//boost::thread::sleep(boost::posix_time::seconds(2))
	cout << "hello" << endl;
	// cout<<code<<endl;
}
return 0;
}
