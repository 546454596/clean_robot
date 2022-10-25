#include <APF.h>

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msgs){
    speed = sqrt(pow(odometry_msgs->twist.twist.linear.x, 2) + pow(odometry_msgs->twist.twist.linear.y, 2));
    position << odometry_msgs->pose.pose.position.x, odometry_msgs->pose.pose.position.y, odometry_msgs->pose.pose.position.z;
    height = odometry_msgs->pose.pose.position.z;

    Eigen::Quaternionf quaternion(odometry_msgs->pose.pose.orientation.w, odometry_msgs->pose.pose.orientation.x, odometry_msgs->pose.pose.orientation.y, odometry_msgs->pose.pose.orientation.z);
    R = quaternion.toRotationMatrix();
}

void attractiveVelocityCallback(const geometry_msgs::TwistStamped& command_msgs){
    velocity_d << command_msgs.twist.linear.x, command_msgs.twist.linear.y, command_msgs.twist.linear.z, command_msgs.twist.angular.z;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 msg_cloud;
    projector.projectLaser(*scan, msg_cloud);
    pcl::fromROSMsg(msg_cloud, *cloud);

    transformation << R(0, 0), R(0, 1), R(0, 2), position(0),
                      R(1, 0), R(1, 1), R(1, 2), position(1),
                      R(2, 0), R(2, 1), R(2, 2), position(2),
                      0,0,0,1;
    pcl::transfomPointCloud(*cloud, *cloud, transformation);
    cloud->header.frame_id = "map";

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.2, 100);
    pass.filter(*cloud);
}

APF::APF(int argc, char** argv) {
    ros::init(argc, argv, "APF");
    ros::NodeHandle node_handle;

    //Subscribers
    odometry_subscriber = node_handle.subscribe("/odom", 1, odometryCallback);
    attractive_velocity_subscriber = node_handle.subscribe("/odom/attrative_velocity", 1, attractiveVelocityCallback);
    laser_subscriber = node_handle.subscribe("/scan", 1, laserCallback);

    //Publishers
    force_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/potential_fields/velocity", 1);
    attractive_publisher = node_handle.advertise<geometry_msgs::Vector3>("/potential_fields/attractive", 1);
    repulsive_publisher = node_handle.advertise<geometry_msgs::Vector3>("/potential_fields/repulsive", 1);
    point_cloud_publisher = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/point_cloud", 1);

    transformation = MatrixXf::Identity(4, 4);
}

APF::~APF() {
    ros::shutdown();
    exit(0);
}

voild APF::run() {
    Vector3f force;
    Vector3f repulsive_force;

    ros::Rate rate(100);
    while(ros::ok()){
        ros::sleep();
        ros::spinOnce();

        repulsive_force << 0, 0, 0;
        int points = 0;
        for(int i = 0; i < cloud->size(); ++i){
            Vector3f obstacle(cloud->points[i].x - position(0), cloud->points[i].y - position(1), 0);
            float eta = distance(obstacle);
            if (eta < eta_0){
                obstacle -= obstacle/eta*UAV_radius;
                repulsive_force += (1/eta - 1/eta_0)/ pow(eta, 2)*obstacle;
                ++points;
            }
        }
        if(points){

        }

        repulsive_force *= k_repulsive;

        force = velocity_d.head(3) - repulsive_force;

        geometry_msgs::TwistStamped force_msg;
        force_msg.header.stamp = ros::Time::now();
        force_msg.twist.linear.x = force(0);
        force_msg.twist.linear.y = force(1);
        force_msg.twist.linear.z = velocity_d(2);
        force_msg.twist.angular.z = velocity_d(3);
        force_publisher.publish(force_msg);

        geometry_msgs::Vector3 debug_msg;
        debug_msg.x = velocity_d(0);
        debug_msg.y = velocity_d(1);
        debug_msg.z = velocity_d(2);
        attractive_publisher.publish(debug_msg);
        debug_msg.x = -repulsive_force(0);
        debug_msg.y = -repulsive_force(1);
        debug_msg.z = 0;
        repulsive_publisher.publish(debug_msg);

        point_cloud_publisher.publish(cloud);
    }
}

double APF::distance(Vector3f p) {Vector3f v}{
    return sqrt(pow(v(0), 2) + pow(v(1), 2) + pow(v(2), 2));
}

int main(int argc, char** argv){
    cout << "[APF] Artificial potential fields running..." << endl;

    APF* apf = new APF(argc, argv);
    apf->run();
}
