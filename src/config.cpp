#include "lems/data/config.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <regex>
#include <stdexcept>

namespace lems::data {
namespace {
std::string trim(std::string s){auto ws=[](unsigned char c){return std::isspace(c);};s.erase(s.begin(),std::find_if_not(s.begin(),s.end(),ws));s.erase(std::find_if_not(s.rbegin(),s.rend(),ws).base(),s.end());if(s.size()>=2&&((s.front()=='"'&&s.back()=='"')||(s.front()=='\''&&s.back()=='\'')))s=s.substr(1,s.size()-2);return s;}
std::vector<double> nums(const std::string&s){std::vector<double>v;static const std::regex r(R"([-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?)");for(std::sregex_iterator i(s.begin(),s.end(),r),e;i!=e;++i)v.push_back(std::stod(i->str()));return v;}
template<size_t N>void copy_values(const std::vector<double>&v,std::array<double,N>&a){if(v.size()!=N)throw std::runtime_error("expected "+std::to_string(N)+" calibration values, got "+std::to_string(v.size()));std::copy(v.begin(),v.end(),a.begin());}
void finish_camera(Camera&c){c.K={c.intrinsics[0],0,c.intrinsics[2],0,c.intrinsics[1],c.intrinsics[3],0,0,1};c.projection={c.K[0],c.K[1],c.K[2],0,c.K[3],c.K[4],c.K[5],0,c.K[6],c.K[7],c.K[8],0};}
std::array<double,9> inv3(const std::array<double,9>&m){double d=m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+m[2]*(m[3]*m[7]-m[4]*m[6]);if(std::abs(d)<1e-15)throw std::runtime_error("singular camera matrix");return {(m[4]*m[8]-m[5]*m[7])/d,(m[2]*m[7]-m[1]*m[8])/d,(m[1]*m[5]-m[2]*m[4])/d,(m[5]*m[6]-m[3]*m[8])/d,(m[0]*m[8]-m[2]*m[6])/d,(m[2]*m[3]-m[0]*m[5])/d,(m[3]*m[7]-m[4]*m[6])/d,(m[1]*m[6]-m[0]*m[7])/d,(m[0]*m[4]-m[1]*m[3])/d};}
std::array<double,9> mul3(const std::array<double,9>&x,const std::array<double,9>&y){std::array<double,9>z{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)for(int k=0;k<3;++k)z[i*3+j]+=x[i*3+k]*y[k*3+j];return z;}
std::array<double,9> make_f(const Camera&a,const Camera&b,const StereoCalibration&s){auto ia=inv3(a.K),ib=inv3(b.K);std::array<double,9>ibt={ib[0],ib[3],ib[6],ib[1],ib[4],ib[7],ib[2],ib[5],ib[8]};auto&t=s.t_target_reference;std::array<double,9>tx={0,-t[2],t[1],t[2],0,-t[0],-t[1],t[0],0};return mul3(mul3(mul3(ibt,tx),s.R_target_reference),ia);}
}

DatasetConfig load_config(const std::filesystem::path&path){
 std::ifstream in(path);if(!in)throw std::runtime_error("cannot open config: "+path.string());DatasetConfig c;Camera left,right;left.name="cam0";right.name="cam1";StereoCalibration st;std::string section,last_key,line;std::vector<double>matrix;
 auto flush=[&]{if(last_key=="R21"&&!matrix.empty())copy_values(matrix,st.R_target_reference);else if(last_key=="F21"&&!matrix.empty())copy_values(matrix,st.fundamental);matrix.clear();last_key.clear();};
 while(std::getline(in,line)){auto hash=line.find('#');if(hash!=std::string::npos)line.resize(hash);if(trim(line).empty())continue;size_t indent=line.find_first_not_of(' ');auto clean=trim(line);auto colon=clean.find(':');
  if(clean.rfind("-",0)==0&&(last_key=="R21"||last_key=="F21")){auto v=nums(clean);matrix.insert(matrix.end(),v.begin(),v.end());continue;}if(colon==std::string::npos)continue;auto key=trim(clean.substr(0,colon)),value=trim(clean.substr(colon+1));if(indent==0&&value.empty()){flush();section=key;continue;}if(indent==0){flush();section.clear();}
  if(section=="left_camera"||section=="right_camera"){auto&cam=section=="left_camera"?left:right;auto v=nums(value);if(key=="resolution"){if(v.size()!=2)throw std::runtime_error("resolution needs 2 values");cam.resolution={int(v[0]),int(v[1])};}else if(key=="intrinsics")copy_values(v,cam.intrinsics);else if(key=="distortion_coefficients")cam.distortion=v;else if(key=="camera_model"||key=="model")cam.model=value;}
  else if(section=="stereo"){if(key=="R21"||key=="F21"){flush();last_key=key;auto v=nums(value);matrix.insert(matrix.end(),v.begin(),v.end());}else if(key=="T21")copy_values(nums(value),st.t_target_reference);}
  else if(key=="dataset_type"||key=="type")c.type=value;else if(key=="dataset_dir"||key=="root")c.root=value;else if(key=="sequence_name"||key=="sequence")c.sequence=value;else if(key=="skip_frames")c.skip_frames=std::stoull(value);else if(key=="sync_tolerance_ns")c.sync_tolerance_ns=std::stoll(value);else if(key=="gt_file_path")c.ground_truth_path=value;else c.options[key]=value;
 }flush();if(c.type.empty()||c.root.empty())throw std::runtime_error("config requires dataset_type and dataset_dir");if(c.root.is_relative())c.root=std::filesystem::weakly_canonical(path.parent_path()/c.root);if(c.ground_truth_path&&c.ground_truth_path->is_relative())*c.ground_truth_path=c.root/ *c.ground_truth_path;
 if(left.intrinsics[0]!=0){finish_camera(left);c.cameras.push_back(left);}if(right.intrinsics[0]!=0){finish_camera(right);c.cameras.push_back(right);}if(c.cameras.size()==2){st.baseline=std::sqrt(st.t_target_reference[0]*st.t_target_reference[0]+st.t_target_reference[1]*st.t_target_reference[1]+st.t_target_reference[2]*st.t_target_reference[2]);bool empty=std::all_of(st.fundamental.begin(),st.fundamental.end(),[](double x){return x==0;});if(empty)st.fundamental=make_f(c.cameras[0],c.cameras[1],st);c.stereo=st;}return c;
}
} // namespace lems::data
