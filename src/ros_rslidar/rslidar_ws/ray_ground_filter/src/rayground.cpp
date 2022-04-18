#include "nodelet/nodelet.h"

#include "pluginlib/class_list_macros.h"
#include "ros/ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#define CLIP_HEIGHT 0.2 //截取掉高于雷达自身0.2米的点
#define MIN_DISTANCE 2.4
#define RADIAL_DIVIDER_ANGLE 0.18
#define SENSOR_HEIGHT 0.3

//同心分割线距离
#define concentric_divider_distance_ 0.01  //0.1 meters default
#define min_height_threshold_ 0.05
#define local_max_slope_ 8   //max slope of the ground between points, degree
#define general_max_slope_ 5 //max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2

namespace rslidar_pre {
    class RayGound: public nodelet::Nodelet {

        private:
            ros::Subscriber sub_point_cloud_;

            // ros::Publisher pub_filtered_points_;

            ros::Publisher pub_ground_, pub_no_ground_;

            struct PointXYZIRTColor
            {
                pcl::PointXYZI point;

                float radius; //cylindric coords on XY Plane
                float theta;  //angle deg on XY plane
            
                size_t radial_div;     //index of the radial divsion to which this point belongs to
                size_t concentric_div; //index of the concentric division to which this points belongs to
            
                size_t original_index; //index of this point in the source pointcloud

            };
            typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

            size_t radial_dividers_num_;
            size_t concentric_dividers_num_;

        public:
            RayGound(){}

            void onInit(){
                ros::NodeHandle nh = getPrivateNodeHandle();

                sub_point_cloud_ = nh.subscribe("/rspoints", 10, &RayGound::point_cb, this);

                pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_ground", 10);
                pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground", 10);

                // ros::spin();
                // while(ros::ok()){
                //     ros::spinOnce();
                // }
            }

            void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr){
                pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

                pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

                clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr);
                pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZI>);

                remove_close_pt(MIN_DISTANCE, cliped_pc_ptr, remove_close);

                PointCloudXYZIRTColor organized_points;
                std::vector<pcl::PointIndices> radial_division_indices;
                std::vector<pcl::PointIndices> closest_indices;
                std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

                radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);

                XYZI_to_RTZColor(remove_close, organized_points,
                                radial_division_indices, radial_ordered_clouds);
                // XYZI_to_RTZColor(cliped_pc_ptr, organized_points,
                //                 radial_division_indices, radial_ordered_clouds);

                pcl::PointIndices ground_indices, no_ground_indices;

                classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);

                pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

                pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
                extract_ground.setInputCloud(remove_close);
                // extract_ground.setInputCloud(cliped_pc_ptr);

                extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

                extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
                extract_ground.filter(*ground_cloud_ptr);

                extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
                extract_ground.filter(*no_ground_cloud_ptr);

                publish_cloud(pub_ground_, ground_cloud_ptr, in_cloud_ptr->header);
                publish_cloud(pub_no_ground_, no_ground_cloud_ptr, in_cloud_ptr->header);
            }

            void publish_cloud(const ros::Publisher &in_publisher,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                                const std_msgs::Header &in_header)
            {
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
                cloud_msg.header = in_header;
                in_publisher.publish(cloud_msg);
            }

            /*!
            * Classifies Points in the PointCoud as Ground and Not Ground
            通过判断射线中前后两点的坡度是否大于我们事先设定的坡度阈值，从而判断点是否为地面点
            * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
            * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
            * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
            */
            void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                                        pcl::PointIndices &out_ground_indices,
                                        pcl::PointIndices &out_no_ground_indices)
            {
                out_ground_indices.indices.clear();     //地面点
                out_no_ground_indices.indices.clear();  //非地面点
            #pragma omp for
                //sweep through each radial division 遍历每一根射线
                for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) 
                {
                    float prev_radius = 0.f;
                    float prev_height = -SENSOR_HEIGHT;
                    bool prev_ground = false;
                    bool current_ground = false;

                    //遍历每个射线上的所有点
                    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div
                    {
                        //与前一个点的距离
                        float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
                        // local_max_slope_，是我们设定的同条射线上邻近两点的坡度阈值
                        float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
                        //当前点的实际z值
                        float current_height = in_radial_ordered_clouds[i][j].point.z;   
                        // general_max_slope_ 表示整个地面的坡度阈值      
                        float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

                        //for points which are very close causing the height threshold to be tiny, set a minimum value
                        // 对于非常接近导致高度阈值很小的点，请设置一个最小值
                        if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
                        {
                            height_threshold = min_height_threshold_;
                        }

                        //check current point height against the LOCAL threshold (previous point)
                        //对照局部阈值检查当前点高度（上一点）
                        if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
                        {
                            //Check again using general geometry (radius from origin) if previous points wasn't ground
                            if (!prev_ground)   //前一点不是地面点，需要进一步判断当前点
                            {
                                if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                                {
                                    current_ground = true;
                                }
                                else
                                {
                                    current_ground = false;
                                }
                            }
                            else  // 上一点是地面点，当前点在上一点阈值高度范围内，所以这一点一定是地面点
                            {
                                current_ground = true;
                            }
                        }
                        else
                        {
                            //check if previous point is too far from previous one, if so classify again
                            // 检查当前点是否离上个点太远，如果是，重新分类
                            if (points_distance > reclass_distance_threshold_ &&
                                (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
                            {
                                current_ground = true;
                            }
                            else
                            {
                                current_ground = false;
                            }
                        }

                        // 最终判断当前点是否为地面点
                        if (current_ground)
                        {
                            out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                            prev_ground = true;
                        }
                        else
                        {
                            out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                            prev_ground = false;
                        }

                        prev_radius = in_radial_ordered_clouds[i][j].radius;
                        prev_height = in_radial_ordered_clouds[i][j].point.z;
                    }
                }
            }

            /*!
            *
            * @param[in] in_cloud Input Point Cloud to be organized in radial segments
            * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
            * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
            * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
            */
            void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                            PointCloudXYZIRTColor &out_organized_points,
                                            std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                            std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
            {
                out_organized_points.resize(in_cloud->points.size());
                out_radial_divided_indices.clear();
                out_radial_divided_indices.resize(radial_dividers_num_);
                out_radial_ordered_clouds.resize(radial_dividers_num_);

                for (size_t i = 0; i < in_cloud->points.size(); i++)
                {
                    PointXYZIRTColor new_point;
                    auto radius = (float)sqrt(
                        in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
                    auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
                    if (theta < 0)
                    {
                        theta += 360;
                    }
                    //角度的微分
                    auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
                    //半径的微分
                    auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

                    new_point.point = in_cloud->points[i];
                    new_point.radius = radius;
                    new_point.theta = theta;
                    new_point.radial_div = radial_div;
                    new_point.concentric_div = concentric_div;
                    new_point.original_index = i;

                    out_organized_points[i] = new_point;

                    //radial divisions更加角度的微分组织射线
                    out_radial_divided_indices[radial_div].indices.push_back(i);

                    out_radial_ordered_clouds[radial_div].push_back(new_point);

                } //end for

                //将同一根射线上的点按照半径（点到雷达的水平距离）排序
            #pragma omp for
                for (size_t i = 0; i < radial_dividers_num_; i++)
                {
                    std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                            [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
                }
            }

            void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
            {
                pcl::ExtractIndices<pcl::PointXYZI> cliper;

                cliper.setInputCloud(in);
                pcl::PointIndices indices;
            #pragma omp for
                for (size_t i = 0; i < in->points.size(); i++)
                {
                    double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

                    if (distance < min_distance)
                    {
                        indices.indices.push_back(i);
                    }
                }
                cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
                cliper.setNegative(true); //ture to remove the indices
                cliper.filter(*out);
            }

            void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
            {
                pcl::ExtractIndices<pcl::PointXYZI> cliper;

                cliper.setInputCloud(in);
                pcl::PointIndices indices;
            #pragma omp for
                for (size_t i = 0; i < in->points.size(); i++)
                {
                    if (in->points[i].z > clip_height)
                    {
                        indices.indices.push_back(i);
                    }
                }
                cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
                cliper.setNegative(true); //ture to remove the indices
                cliper.filter(*out);
            }
        

    };

};

PLUGINLIB_EXPORT_CLASS(rslidar_pre::RayGound,nodelet::Nodelet)