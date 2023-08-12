#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

// 外参传递时，直接通过nh.param参数加载赋给实例化的对象
Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;           // 有效点集合,大于10m则是盲区
  N_SCANS   = 6;            // 多线激光雷达的线数
  SCAN_RATE = 10;
  group_size = 8;           // 8个点为一组
  disA = 0.01;              // 点集合的距离阈值,判断是否为平面
  disB = 0.1; // B?         // 点集合的距离阈值,判断是否为平面 diaB=0.1?
  p2l_ratio = 225;          // 点到线的距离阈值，需要大于这个值才能判断组成面
  limit_maxmid =6.25;       // 中点到左侧的距离变化率范围
  limit_midmin =6.25;       // 中点到右侧的距离变化率范围
  limit_maxmin = 3.24;      // 左侧到右侧的距离变化率范围
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;                // 点与点距离超过两倍则认为遮挡
  edgeb = 0.1;              // 点与点距离超过0.1m则认为遮挡
  smallp_intersect = 172.5; // 三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面
  smallp_ratio = 1.2;
  given_offset_time = false;  //是否提供时间偏移

  jump_up_limit = cos(jump_up_limit/180*M_PI);        //角度大于170度的点跳过
  jump_down_limit = cos(jump_down_limit/180*M_PI);    //角度小于8度的点
  cos160 = cos(cos160/180*M_PI);                      //夹角限制
  smallp_intersect = cos(smallp_intersect/180*M_PI);  //三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面
}

Preprocess::~Preprocess() {}

// 设置初始化参数，nh.param已赋值
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

/**
 * @brief Livox激光雷达点云预处理函数
 *
 * @param msg livox激光雷达点云数据，格式为livox_ros_driver::CustomMsg
 * @param pcl_out 输出处理后的点云数据，格式为pcl::PointCloud<pcl::PointXYZINormal>
 */
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;
  
  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

/*
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
#################################################
#CustomPoint[]	
uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
*/
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  // 清除之前的点云缓存
  pl_surf.clear();             
  pl_corn.clear();             
  pl_full.clear();             
  double t1 = omp_get_wtime(); 
  int plsize = msg->point_num; // 一帧中的点云总个数
  // cout<<"plsie: "<<plsize<<endl;

  //点云预留空间
  pl_corn.reserve(plsize);  //角点
  pl_surf.reserve(plsize);  //面点
  pl_full.resize(plsize);   //储存全部点，特征或者间隔采样后

  // 根据线束，初始化线束数组的空间
  for(int i=0; i<N_SCANS; i++)
  {
    // 存储每一根线的点云 PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;
  
  // 开启特征提取
  if (feature_enabled)
  {
    // 遍历每一个点
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points，curvature unit: ms

        bool is_new = false;
        // 前后两点不重合
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          // 按照line划分点云，根据同一线束上的点提取特征，保存到线束数组中
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count ++;
    double t0 = omp_get_wtime();
    // 对每个line中的激光雷达分别进行处理
    for(int j=0; j<N_SCANS; j++)
    { 
      // // 如果该line中的点云过小，则继续处理下一条line
      if(pl_buff[j].size() <= 5) continue;
      // 当前线束中的点云pl
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      // 当前线束中激光点类型容器，建立了一个容器数组，储存每个点代表的类型，记录了距离，角度，特征
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for(uint i=0; i<plsize; i++)
      {
        // 计算每个点xy平面距离
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        // 计算两个间隔点的距离
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      // //因为i最后一个点没有i+1了所以就单独求了一个range，没有distance
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      // 传入同一线束点云和距离、间距属性，计算特征
      give_feature(pl, types);
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  // 不进行特征提取，全点云，构建点面残差
  else
  {
    //遍历msg中所有点，并存储到pl_surf中
    for(uint i=1; i<plsize; i++)
    {
      // 只取线数在0~N_SCANS内并且回波次序为0或者1的点云
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;
        // 间隔滤波
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

          if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
              || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  // 特征提取
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;
      if(pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  // 非特征提取
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end  = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    if(feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }
      
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        int layer  = pl_orig.points[i].ring;
        if (layer >= N_SCANS) continue;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // units: ms

        if (!given_offset_time)
        {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        pl_buff[layer].points.push_back(added_pt);
      }

      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) continue;
        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl, types);
      }
    }
    else
    {
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

        if (!given_offset_time)
        {
          int layer = pl_orig.points[i].ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        if (i % point_filter_num == 0)
        {
          if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
          {
            pl_surf.points.push_back(added_pt);
          }
        }
      }
    }
}

// pl 一条line的点云信息，该line点云每点的属性
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();  //单条线的点数
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;
  //不能在盲区 从这条线非盲区的点开始
  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  // group_size默认等于8  plsize2 = plsize - group_size
  // 点云总数要有8个点冗余
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero()); //当前平面的法向量
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero()); //上一个平面的法向量

  uint i_nex = 0, i2;                       // i2为当前点的下一个点
  uint last_i = 0; uint last_i_nex = 0;     // last_i为上一个点的保存的索引 // last_i_nex为上一个点的下一个点的索引
  int last_state = 0;                       // 为1代表上个状态为平面 否则为0
  int plane_type;                           // 判断面点

  // 判断平面特征   
  for(uint i=head; i<plsize2; i++)
  {
    // 在盲区范围内的点不做处理
    if(types[i].range < blind)
    {
      continue;
    }
    // i2记录当前点索引
    i2 = i;
    // i_nex 局部最后一个点的索引
    // curr_direct 归一化后，局部范围最后一个点与第一个点的坐标差值，即向量 i_cur --> i_nex
    //求得平面，并返回类型0 1平面 2
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
    //返回1一般默认是平面
    if(plane_type == 1)
    {
      //设置确定的平面点和可能的平面点
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          // 局部范围内部点设置为确定的平面点
          types[j].ftype = Real_Plane;
        }
        else
        {
          // 局部范围边界点设置为可能的平面点
          types[j].ftype = Poss_Plane;
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5) 
      //如果之前状态是平面则判断当前点是处于两平面边缘的点还是较为平坦的平面的点
      // 根据上一状态，平面或者局部以及长度，向量模长，决定起始点类型
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
        {
          //修改ftype，两个面法向量夹角在45度和135度之间 认为是两平面边缘上的点
          types[i].ftype = Edge_Plane;
        }
        else
        {
          //否则认为是真正的平面点
          types[i].ftype = Real_Plane;
        }
      }
      
      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0; //设置为不是平面点
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }
    // 更新last状态
    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }
  // 头三个和尾三个留出
  //判断边缘点 //如果剩下的点数小于3则不判断边缘点，否则计算哪些点是边缘点
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  { //点不能在盲区 或者 点必须属于正常点和可能的平面点
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }
    //该点与前后点的距离不能挨的太近
    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }
    //当前点组成的向量
    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    // 当前点指向前后相邻点的两个向量
    Eigen::Vector3d vecs[2];  

    // 更新当前点与前后两点的属性 保存到types[i].edj
    // 计算当前点与后一点和前一点的向量，判断当前点前后两个方向的edge属性
    for(int j=0; j<2; j++)
    {
      // 前一点 m=-1, 后一点 m=1
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }
      //若当前的前/后一个点在盲区内
      if(types[i+m].range < blind)
      { 
        //若当前点大于10m
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;   //该点相邻点为无穷大点，跳变较远
        }
        // 当前点小于10m
        else
        {
          types[i].edj[j] = Nr_blind; //该点的相邻点在盲区
        }
        // 下一个相邻点，即后一点
        continue;
      }

      // 相邻点不在盲区
      // 计算相邻点的向量
      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      // 当前点指向相邻点的向量
      vecs[j] = vecs[j] - vec_a; 
      //假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
      // 这个是角OAM和角OAN的cos值
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)   //小于170°
      {
        types[i].edj[j] = Nr_180;
      }
      else if(types[i].angle[j] > jump_down_limit)    // 大于8度
      {
        types[i].edj[j] = Nr_zero;
      }
    }
    //若雷达坐标系原点为O 当前点为A 前/后一点为M和N
    //则下面OA点乘MA/（|OA|*|MA|）
    //得到的是cos角OAM的大小
    //角MAN的cos值
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    
    // 根据前点edge jump属性，后一点edge jumo属性
    // 前一个点是正常点 && 下一个点在激光线上 && 当前点与后一个点的距离大于0.0225m && 当前点与后一个点的距离大于当前点与前一个点距离的四倍
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160) // 160度
      {
        // 判断是否式边缘点
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
     
    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
    
      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  // 0.01*sqrt(x^2+y^2)+0.1 基本上可以近似看成是0.1 100m的时候才到0.2
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;  //前后点距离数组 间距数组
  disarr.reserve(20);
  //距离小 点与点之间较近 先取够8个点
  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();  //距离雷达原点太小了将法向量设置为零向量
      return 2;
    }
    disarr.push_back(types[i_nex].dista); //存储当前点与后一个点的距离
  }
  // i_nex = i_cur+group_size;
  //继续向后遍历，保存group_dis范围内，与后一个点的间距，记录最后一个局部点索引为i_nex
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero(); //距离雷达原点太小,将法向量设置为零向量
      return 2;
    }
    // 计算后面的点，与当前点距离的平方和
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    // 超出局部范围距离约束，退出循环  
    if(two_dis >= group_dis)
    {
      break;
    }
    //存储当前点与后一个点的距离
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;                // 记录局部范围内，叉乘的最大模长，即局部范围内最大平行四边形面积
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)   // 局部范围内，从当前点向后遍历，i_nex为局部范围索引的最大值
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    //假设i_cur点为A j点为B i_nex点为C
    //向量AB
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;
    // vx,vy,vz为 AC
    //向量AB叉乘向量AC
    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];
    //物理意义是组成的ABC组成的平行四边形面积的平方(为|AC|*h，其中为B到线AC的距离)
    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    // 寻找最大面积的平方 
    if(lw > leng_wid)
    {
      leng_wid = lw;  //寻找最大面积的平方(也就是寻找距离AC最远的B)
    }
  }

  // AC*AC/S^2=1/(h*h)  p2l_ratio = 225;       
  // 即 h越大，不能成为平面 当h>1/15m=0.0667m,距离太远了   
  // 点到线的距离阈值，需要大于这个值才能判断组成面
  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();  //太近了法向量直接设置为0
    return 0;
  }

  // 排序，disarr按从大到小排序
  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }
  // 第二小的点间距太小，也设置为0向量
  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==AVIA)
  {
    //点与点之间距离变化太大的时候 可能与激光束是平行的 就也舍弃
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  
  // 储存向量AC
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

//边缘判断
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)  //前两个点不能在盲区
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)  //后两个点不能在盲区
    {
      return false;
    }
  }

  //下面分别对i-2 i-1和i i+1两种情况时点与点间距进行了判断
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;
  //将大小间距进行调换 大在前 小在后
  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    //假如间距太大 可能是被遮挡，就不把它当作边缘点
    return false;
  }
  
  return true;
}
