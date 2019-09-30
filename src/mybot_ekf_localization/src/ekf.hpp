#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/nondet_random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>

// Point cloud library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

// Bayesian filtering library
#include <filter/extendedkalmanfilter.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

//include aggiuntivi
#include "bayesian_filtering/nonlinearanalyticconditionalgaussianmobile.h"
#include <laser_geometry/laser_geometry.h>


class ROSNode
{
    //elemento base delle pointcloud
    typedef pcl::PointXYZ point_type;

    ros::NodeHandle node_handle;

    bool map_state = false;
    bool laser_state = false;
    bool odom_state = false;

    double x_init = 0.0;
    double y_init = 0.0;
    double theta_init = 0.0;


    //grandezze dei modelli gaussiani e del filtro
    BFL::ColumnVector input;
    BFL::ColumnVector odom; //per memorizzare odometria di gazebo
    BFL::ColumnVector prediction; //per memorizzare predizione
    boost::shared_ptr<BFL::NonLinearAnalyticConditionalGaussianMobile> system_pdf;
    boost::shared_ptr<BFL::AnalyticSystemModelGaussianUncertainty> system_model;
    boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> measurement_pdf;
    boost::shared_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> measurement_model;
    boost::shared_ptr<BFL::ExtendedKalmanFilter> filter;


    //parametri per esecuzione iterative closest point
    double voxel_grid_size = 0.005;
    double max_correspondence_distance = 100.0;
    double max_iterations = 1000;
    double ransac_iterations = 1000;
    double ransac_outlier_threshold = 0.1;
    double icp_optimization_epsilon = 0.0000001;
    double icp_score_scale = 100.0;

    ros::Time odom_last_stamp_, laser_stamp, odom_init_stamp_, filter_stamp_;


    //sistemi di riferimento
    std::string base_frame;
    std::string odom_frame;
    std::string map_frame;
    std::string laser_frame;
    tf::Stamped<tf::Pose> new_transform;
    tf::Stamped<tf::Pose> odom_to_map;
    tf::TransformBroadcaster transform_broadcaster;


    //Publischer e Subscriber
    ros::Subscriber laser_sub;
    ros::Subscriber map_sub_;
    ros::Subscriber cmdvel_sub;
    ros::Subscriber odom_sub;

    ros::Publisher map_pub_;
    ros::Publisher laser_pub_;
    ros::Publisher draw_position;
    ros::Publisher position_error;
    boost::shared_ptr<tf::TransformListener> listener;


    //Timer
    ros::Timer timer;
    ros::Time time_ = ros::Time::now();


    //elementi per le pointcloud
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<point_type>::Ptr map_cloud;
    pcl::PointCloud<point_type>::Ptr laser_cloud;
    laser_geometry::LaserProjection projector;
    pcl::IterativeClosestPoint<point_type, point_type> icp;


    //metodi
    void initKalmanFilter();
    void predictStep();
    void correctStep();
    void drawCovariance(const Eigen::Matrix2f& covMatrix);
    void broadcast_tf(const ros::Time& broad_cast_time);
    void angleOverflowCorrect(double& a);


    //callbacks
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid& map_msg);
    void input_callback(const geometry_msgs::Twist& input_msg);
    void timer_callback(const ros::TimerEvent& e);
    void odom_callback(const nav_msgs::Odometry& input_msg);


    //costruttore
    public:

        ROSNode(const ros::NodeHandle& nh, const double & spin_rate);//costruttore

};
