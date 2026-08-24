#include "lems/data/dataset.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fs=std::filesystem;
namespace lems::data {
namespace {

std::vector<fs::path> images_in(const fs::path& dir) {
  std::vector<fs::path> out;
  if(!fs::exists(dir)) return out;
  for(const auto& e:fs::directory_iterator(dir)) if(e.is_regular_file()) {
    auto x=e.path().extension().string(); std::transform(x.begin(),x.end(),x.begin(),::tolower);
    if(x==".png"||x==".jpg"||x==".jpeg"||x==".pgm") out.push_back(e.path());
  }
  std::sort(out.begin(),out.end()); return out;
}

Timestamp seconds_to_ns(double x){return static_cast<Timestamp>(std::llround(x*1e9));}

std::vector<Timestamp> timestamps_file(const fs::path& p) {
  std::vector<Timestamp> v; std::ifstream in(p); std::string s;
  while(std::getline(in,s)){ if(s.empty()||s[0]=='#') continue; std::istringstream q(s); double t; if(q>>t)v.push_back(seconds_to_ns(t)); }
  return v;
}

std::vector<double> line_values(const std::string& line) {
  std::vector<double> out; auto p=line.find(':'); std::istringstream in(p==std::string::npos?line:line.substr(p+1));
  std::string token; while(in>>token){token.erase(std::remove_if(token.begin(),token.end(),[](char c){return c=='['||c==']'||c==','; }),token.end());try{if(!token.empty())out.push_back(std::stod(token));}catch(...) {}}
  return out;
}

std::vector<std::array<double,12>> kitti_poses(const fs::path& p) {
  std::vector<std::array<double,12>> out; std::ifstream in(p); std::string line;
  while(std::getline(in,line)){std::istringstream q(line);std::array<double,12>a{};bool ok=true;for(auto&x:a)if(!(q>>x)){ok=false;break;}if(ok)out.push_back(a);}return out;
}

void load_kitti_calibration(const fs::path&path,DatasetConfig&c){
 std::ifstream in(path);if(!in)return;std::array<double,12>p0{},p1{};bool h0=false,h1=false;std::string line;
 while(std::getline(in,line)){if(line.rfind("P0:",0)==0||line.rfind("P2:",0)==0){auto v=line_values(line);if(v.size()==12){std::copy(v.begin(),v.end(),p0.begin());h0=true;}}else if(line.rfind("P1:",0)==0||line.rfind("P3:",0)==0){auto v=line_values(line);if(v.size()==12){std::copy(v.begin(),v.end(),p1.begin());h1=true;}}}
 if(!h0||!h1)return;Camera a,b;a.name="cam0";b.name="cam1";a.projection=p0;b.projection=p1;a.K={p0[0],p0[1],p0[2],p0[4],p0[5],p0[6],p0[8],p0[9],p0[10]};b.K={p1[0],p1[1],p1[2],p1[4],p1[5],p1[6],p1[8],p1[9],p1[10]};a.intrinsics={a.K[0],a.K[4],a.K[2],a.K[5]};b.intrinsics={b.K[0],b.K[4],b.K[2],b.K[5]};c.cameras={a,b};StereoCalibration s;s.t_target_reference={p1[3]/p1[0]-p0[3]/p0[0],0,0};s.baseline=std::abs(s.t_target_reference[0]);c.stereo=s;
}

std::array<double,4> rotation_to_quaternion(const std::array<double,12>&p){
  double tr=p[0]+p[5]+p[10],w,x,y,z;if(tr>0){double s=std::sqrt(tr+1.0)*2;w=.25*s;x=(p[9]-p[6])/s;y=(p[2]-p[8])/s;z=(p[4]-p[1])/s;}else if(p[0]>p[5]&&p[0]>p[10]){double s=std::sqrt(1+p[0]-p[5]-p[10])*2;w=(p[9]-p[6])/s;x=.25*s;y=(p[1]+p[4])/s;z=(p[2]+p[8])/s;}else if(p[5]>p[10]){double s=std::sqrt(1+p[5]-p[0]-p[10])*2;w=(p[2]-p[8])/s;x=(p[1]+p[4])/s;y=.25*s;z=(p[6]+p[9])/s;}else{double s=std::sqrt(1+p[10]-p[0]-p[5])*2;w=(p[4]-p[1])/s;x=(p[2]+p[8])/s;y=(p[6]+p[9])/s;z=.25*s;}return {w,x,y,z};
}

struct Sample { Timestamp t; fs::path path; };
std::vector<Sample> euroc_csv(const fs::path& p, const fs::path& data_dir) {
  std::vector<Sample> v; std::ifstream in(p); std::string s;
  while(std::getline(in,s)){ if(s.empty()||s[0]=='#')continue; std::replace(s.begin(),s.end(),',',' '); std::istringstream q(s); Timestamp t; std::string f; if(q>>t>>f)v.push_back({t,data_dir/f}); }
  return v;
}

class VectorIterator final:public DatasetIterator {
 public: VectorIterator(std::vector<FrameSet> x,std::size_t skip):sets_(std::move(x)),step_(skip+1){}
  std::optional<FrameSet> next() override {if(i_>=sets_.size())return {}; auto x=sets_[i_]; i_+=step_; return x;}
  void reset() override{i_=0;}
 private:std::vector<FrameSet> sets_;std::size_t i_{0},step_{1};
};

class BuiltinDataset final:public Dataset {
 public: explicit BuiltinDataset(DatasetConfig c):c_(std::move(c)){load();}
  const DatasetConfig& config()const override{return c_;}
  const std::vector<Camera>& cameras()const override{return c_.cameras;}
  const std::optional<StereoCalibration>& stereo_calibration()const override{return c_.stereo;}
  std::unique_ptr<DatasetIterator> iterate()const override{return std::make_unique<VectorIterator>(sets_,c_.skip_frames);}
 private:
  void load_kitti(){
    fs::path base=c_.root/c_.sequence;if(!fs::exists(base/"image_0"))base=c_.root/"sequences"/c_.sequence;
    load_kitti_calibration(base/"calib.txt",c_);
    auto l=images_in(base/"image_0"),r=images_in(base/"image_1"); auto ts=timestamps_file(base/"times.txt");
    fs::path gt;if(c_.ground_truth_path){gt=*c_.ground_truth_path;if(fs::is_directory(gt))gt/=fs::path(c_.sequence).filename().string()+".txt";}else gt=c_.root/"poses"/(fs::path(c_.sequence).filename().string()+".txt");auto poses=kitti_poses(gt);
    const auto n=std::min(l.size(),r.size()); for(size_t i=0;i<n;++i){Timestamp t=i<ts.size()?ts[i]:static_cast<Timestamp>(i);std::optional<Pose>p;if(i<poses.size())p=Pose{t,{poses[i][3],poses[i][7],poses[i][11]},rotation_to_quaternion(poses[i])};sets_.push_back({i,t,{{"cam0",t,l[i],p,{}},{"cam1",t,r[i],p,{}}}});}
  }
  void load_euroc(){
    fs::path b=c_.root/c_.sequence/"mav0"; auto l=euroc_csv(b/"cam0/data.csv",b/"cam0/data"),r=euroc_csv(b/"cam1/data.csv",b/"cam1/data"); size_t j=0;
    for(const auto& a:l){while(j+1<r.size()&&std::llabs(r[j+1].t-a.t)<std::llabs(r[j].t-a.t))++j;if(j<r.size()&&std::llabs(r[j].t-a.t)<=c_.sync_tolerance_ns)sets_.push_back({sets_.size(),a.t,{{"cam0",a.t,a.path,{}},{"cam1",r[j].t,r[j].path,{}}}});}
  }
  void load_eth3d(){
    fs::path b=c_.root/c_.sequence;
    if(fs::exists(b/"stereo_pairs")){for(const auto& e:fs::directory_iterator(b/"stereo_pairs"))if(e.is_directory()){auto a=e.path()/"im0.png",d=e.path()/"im1.png";if(fs::exists(a)&&fs::exists(d)){FrameMetadata lm,rm;auto ld=e.path()/"disp0GT.pfm",rd=e.path()/"disp1GT.pfm",lo=e.path()/"mask0nocc.png",ro=e.path()/"mask1nocc.png";if(fs::exists(ld))lm.disparity_path=ld;if(fs::exists(rd))rm.disparity_path=rd;if(fs::exists(lo))lm.occlusion_mask_path=lo;if(fs::exists(ro))rm.occlusion_mask_path=ro;sets_.push_back({0,0,{{"cam0",0,a,{},lm},{"cam1",0,d,{},rm}}});}}std::sort(sets_.begin(),sets_.end(),[](auto&a,auto&b){return a.frames[0].image_path<b.frames[0].image_path;});}
    else {auto l=images_in(b/"rgb"),r=images_in(b/"rgb2"); auto n=std::min(l.size(),r.size());for(size_t i=0;i<n;++i)sets_.push_back({i,static_cast<Timestamp>(i),{{"cam0",static_cast<Timestamp>(i),l[i],{}},{"cam1",static_cast<Timestamp>(i),r[i],{}}}});}
    for(size_t i=0;i<sets_.size();++i){sets_[i].index=i;sets_[i].timestamp_ns=static_cast<Timestamp>(i);for(auto&f:sets_[i].frames)f.timestamp_ns=sets_[i].timestamp_ns;}
  }
  void load(){auto t=c_.type;std::transform(t.begin(),t.end(),t.begin(),::tolower);if(t.find("kitti")!=std::string::npos)load_kitti();else if(t.find("euroc")!=std::string::npos)load_euroc();else if(t.find("eth3d")!=std::string::npos)load_eth3d();else throw std::invalid_argument("unsupported dataset type: "+c_.type);if(sets_.empty())throw std::runtime_error("no synchronized stereo frames found under "+c_.root.string());}
  DatasetConfig c_;std::vector<FrameSet> sets_;
};
}
std::unique_ptr<Dataset> open_dataset(DatasetConfig c){return std::make_unique<BuiltinDataset>(std::move(c));}
} // namespace lems::data
