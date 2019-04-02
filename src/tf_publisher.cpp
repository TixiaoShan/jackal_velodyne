#include "utility.h"


class SimulationJackal{

private:

    ros::NodeHandle nh;

    ros::ServiceClient setClient;
    ros::ServiceClient getClient;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::GetModelState getmodelstate;

    tf::TransformBroadcaster tfMap2OdomBroadcaster;
    
public:

    SimulationJackal(){

        // Gazebo client
        setClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        getClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        // Set Gazebo state
        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = "jackal";
        modelstate.reference_frame = "world";
        setmodelstate.request.model_state = modelstate;

        // Get Gazebo state
        getmodelstate.request.model_name = "jackal";
        getmodelstate.request.relative_entity_name = "world";
    }

    void run(){

        ros::Rate rate(100);

        while (ros::ok()){

            // sleep
            rate.sleep();

            // Get tf between odom and base_link
            try{listener.lookupTransform("odom","base_link", ros::Time(0), transform);}
            catch (tf::TransformException ex){continue;}
            tf::Transform odom_to_laser(transform.getBasis(), transform.getOrigin());

            // get robot pose from Gazebo
            getClient.waitForExistence();
            getClient.call(getmodelstate);

            // get translation
            double x, y, z;
            x = getmodelstate.response.pose.position.x;
            y = getmodelstate.response.pose.position.y;
            z = getmodelstate.response.pose.position.z;

            // get orientation
            geometry_msgs::Quaternion geoQuat = getmodelstate.response.pose.orientation;
            tf::Quaternion tfQuat(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w);

            // calculate tf between map and odom
            tf::Transform laser_to_map = tf::Transform(tfQuat, tf::Vector3(x, y, z)).inverse();
            tf::Transform map_to_odom = (odom_to_laser * laser_to_map).inverse();

            // Publish TF (map to odom)
            tfMap2OdomBroadcaster.sendTransform(tf::StampedTransform(map_to_odom, ros::Time::now(), "/map", "/odom"));
        }
    }
};




int main(int argc, char** argv){
    
    ros::init(argc, argv, "jackal_velodyne");
    
    SimulationJackal simJackal;

    ROS_INFO("\033[1;32m---->\033[0m Gazebo TF publisher.");

    simJackal.run();

    return 0;
}
