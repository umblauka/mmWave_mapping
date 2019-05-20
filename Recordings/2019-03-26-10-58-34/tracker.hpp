#ifndef EXAMPLE_HPP
#define EXAMPLE_HPP

#ifdef __cplusplus
extern "C" {
#endif
typedef enum
{
  TRACKING_DEFAULT_PARAM_SET = 0,
  TRACKING_TRAFFIC_MONITORING_PARAM_SET,
  TRACKING_PEOPLE_COUNTING_PARAM_SET
} TRACKING_ADVANCED_PARAM_SET;

typedef enum
{
  TRACKING_PARAM_SET_TM = 0,
  TRACKING_PARAM_SET_PC
} TRACKING_ADVANCED_PARAM_SET_TABLE;

/* This test application (traffic monitoring), wants to modify default parameters */
GTRACK_sceneryParams appSceneryParamTable[2] = {
    1, {{-7.f, 15.f, 10.f, 185.f}, {0.f, 0.f, 0.f, 0.f}}, 1, {{-6.f, 14.f, 15.f, 60.f}, {0.f, 0.f, 0.f, 0.f}}, /* boundary box: {-1,12,15,75}, static box {0,11,19,50}, both in (left x, right x, bottom y, top y) format */
    0,
    {{0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f}},
    0,
    {{0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f}} /* no boundary boxes, static boxes */
};
GTRACK_gatingParams appGatingParamTable[2] = {
    {16.f, {12.f, 8.f, 0.f}}, /* TM: 16 gating volume, Limits are set to 8m in length, 2m in width, no limit in doppler */
    {2.f, {2.f, 2.f, 0.f}}    /* PC: 2 gating volume, Limits are set to 2m in length, 2m in width, no limit in doppler */
};
GTRACK_stateParams appStateParamTable[2] = {
    {3U, 3U, 5U, 1000U, 5U},  /* TM: 3 det2act, 3 det2free, 5 act2free, 1000 stat2free, 5 exit2free*/
    {2U, 5U, 100U, 1000U, 5U} /* PC: 3 det2act, 5 det2free, 100 act2free, 1000 stat2free, 5 exit2free */
};
GTRACK_allocationParams appAllocationParamTable[2] = {
    {250.f, 100.f, 0.1f, 5U, 2.8f, 2.f}, /* TM: any SNRs, 0m/s minimal velocity,  5 points with 4m in distance, 2m/c in velocity  separation */
    {0.f, 0.f, 0.1f, 2U, 1.f, 2.f}  /* PC: SNRs 150 (250 obscured), 0.1 m/s minimal velocity, 5 points, with 1m in distance, 2m/c in velocity in separation */
};
/* Using standard deviation of uniformly distributed variable in the range [a b]: 1/sqrt(12)*(b-a) */
GTRACK_varParams appVariationParamTable[2] = {
    /* Standard deviation of uniformly distributed number in range [a b]: sqrt(1/12)*(b-a) */
    {4.f / 3.46f, 1.5f / 3.46f, 1.f}, /* TM: 1m height, 1m in width, 2 m/s for doppler */
    {2.f / 3.46f, 2.f / 3.46f, 1.f}   /* PC: 1m height, 1m in width, 1 m/s for doppler */
};

#ifdef __cplusplus
}
#endif

class GTRACK_targetDesc_cpp : public GTRACK_targetDesc {
// a portable solution, offers operator overloading to C struct
public:
  bool operator<(const GTRACK_targetDesc& t) const
  {
      return (this->tid < t.tid);
  }
};

class GTrack_trackHeartbeat{ 
public:
  ros::Time begin;
  uint32_t tid;
  GTRACK_targetDesc_cpp track;
  bool operator<(const GTrack_trackHeartbeat& t) const
  {
      return (this->tid < t.tid);
  }
  bool operator==(const GTrack_trackHeartbeat& t) const
  {
      return (this->tid == t.tid);
  }
};

class Listener
{
private:

  TRACKING_ADVANCED_PARAM_SET trackingParamSet_ = TRACKING_PEOPLE_COUNTING_PARAM_SET;
  GTRACK_moduleConfig config_;
  GTRACK_advancedParameters advParams_;
  GTRACK_measurementPoint points_[250];
  GTRACK_measurement_vector var_[250];
  GTRACK_targetDesc_cpp targetDescr_[20];
  std::vector<GTrack_trackHeartbeat> activeTracksList_;
  void *hTracker_;
  // uint8_t mIndex[250];
  uint16_t mNum_;
  uint16_t tNum_;
  int32_t errCode_;
  uint32_t timeStart_;
  uint32_t benchmarks_[20];
  void drawTrack(GTRACK_targetDesc_cpp targetDescr_);
  void collectAndUpdateActiveTracks();

public:
  ros::Publisher position_pub;
  void create_tracker();
  void publishMarker(uint32_t shape, std::string frame_id, uint32_t marker_id, uint32_t tid, float posX, float posY, float red, float blue, float green);
  void publishAxisMarkers(std::string frame_id, uint32_t marker_id, std::string marker_label, float posX, float posY, float posZ);
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud; //Z is used for velocity
  void callback(const PointCloud::ConstPtr &msg);
  void setConfig(TRACKING_ADVANCED_PARAM_SET trackingParamSet, GTRACK_moduleConfig config);
};



#endif /* EXAMPLE_HPP */