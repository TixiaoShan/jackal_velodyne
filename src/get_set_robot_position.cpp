#include "utility.h"

class SimulationJackal{

private:

    ros::NodeHandle nh;

    ros::ServiceClient setClient;
    ros::ServiceClient getClient;

    gazebo_msgs::ModelState modelstate;

    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::GetModelState getmodelstate;
    
public:

    SimulationJackal(){

        // Gazebo client
        setClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        getClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        // Set Gazebo state
        modelstate.model_name = "jackal";
        modelstate.reference_frame = "world";
        setmodelstate.request.model_state = modelstate;

        // Get Gazebo state
        getmodelstate.request.model_name = "jackal";
        getmodelstate.request.relative_entity_name = "world";
    }

    void getRobotState(){

        // get robot pose from Gazebo
        getClient.call(getmodelstate);

        // get translation
        double x, y, z;
        x = getmodelstate.response.pose.position.x;
        y = getmodelstate.response.pose.position.y;
        z = getmodelstate.response.pose.position.z;
        
        // get rotation
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = getmodelstate.response.pose.orientation;
        tf::Quaternion tfQuat(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w);
        tf::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
    }

    void setRobotState(){

        geometry_msgs::Pose thisPose;

        // set translation
        thisPose.position.x = 0;
        thisPose.position.y = 0;
        thisPose.position.z = 0;

        // set rotation
        thisPose.orientation.x = 0;
        thisPose.orientation.y = 0;
        thisPose.orientation.z = 0;
        thisPose.orientation.w = 1;

        // set gazebo msg state
        modelstate.pose = thisPose;
        setmodelstate.request.model_state = modelstate;

        // call client
        setClient.call(setmodelstate);
    }

};




int main(int argc, char** argv){
    
    ros::init(argc, argv, "jackal_velodyne");
    
    SimulationJackal simJackal;

    ROS_INFO("\033[1;32m---->\033[0m Get or Set Robot Position.");

    return 0;
}
