#include<iostream>
#include<algorithm>
#include<fstream>
#include<sstream>
#include<chrono>
#include<unistd.h>
#include<vector>

#include<System.h>

using namespace std;



void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
				vector<double> &vTimestamps);


int main(int argc, char const *argv[])
{
	if(argc != 4)
	{
		cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
		return 1;
	}
	
	vector<string> vstrImageFilenames;
	vector<double> vTimestamps;
	string strFile = string(argv[3]) + "/rgb.txt";
	LoadImages(strFile, vstrImageFilenames, vTimestamps);
	
	int nImages = vstrImageFilenames.size();
	
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
	
	vector<float> vTimesTrack;
	vTimesTrack.resize(nImages);
	
	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;
	cout << "Images in the sequence: " << nImages << endl << endl;
	
	// Mian loop
	cv::Mat im;
	for (int ni = 0; ni < nImages; ni++)
	{
		im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
		double tframe = vTimestamps[ni];
		
		if(im.empty())
		{
			cerr << endl << "Failed to load image at: "
					<< string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
			return 1;
		}
		
#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
	
		// SLAM.TrackMonocular(im, tframe);
		
	}
	

	
	
	
	
	return 0;
}


/**
 * @brief 导入图片
 * 
 * @param[in] strFile                   读入的文件名称
 * @param[in&out] vstrImageFilenames    彩色图片名称
 * @param[in&out] vTimestamps           记录时间戳
 */
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
	ifstream f;
	f.open(strFile.c_str());

	// skip first three lines
	// 前三行是注释，跳过
	string s0;
	getline(f,s0);
	getline(f,s0);
	getline(f,s0);

	while(!f.eof())
	{
		string s;
		getline(f,s);
		if(!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			string sRGB;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			vstrImageFilenames.push_back(sRGB);
		}
	}
}
