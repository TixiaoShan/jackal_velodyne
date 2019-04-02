#include "utility.h"

class SimulationJackal{

private:

    ros::NodeHandle nh;

    ros::ServiceClient setClient;
    ros::ServiceClient getClient;
    ros::ServiceClient setPhysicsClient;
    ros::ServiceClient getPhysicsClient;

    gazebo_msgs::ModelState modelstate;

    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::GetModelState getmodelstate;
    
public:

    SimulationJackal(){

        // Gazebo client
        setClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        getClient = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        setPhysicsClient = nh.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
        getPhysicsClient = nh.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");

        // Set Gazebo state
        modelstate.model_name = "jackal";
        modelstate.reference_frame = "world";
        setmodelstate.request.model_state = modelstate;

        // Get Gazebo state
        getmodelstate.request.model_name = "jackal";
        getmodelstate.request.relative_entity_name = "world";
    }

    void getRobotState(){

        getClient.waitForExistence();

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

        setClient.waitForExistence();

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

    void setGazeboPhysics(){

        setPhysicsClient.waitForExistence();
        
        // http://docs.ros.org/jade/api/geometry_msgs/html/msg/Vector3.html
        geometry_msgs::Vector3 gravity;
        gravity.x = 0;
        gravity.y = 0;
        gravity.z = -9.81;

        // configurations for ODE , http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ODEPhysics.html
        gazebo_msgs::ODEPhysics ode_config;  
        ode_config.auto_disable_bodies = false;
        ode_config.sor_pgs_precon_iters = 0;
        ode_config.sor_pgs_iters = 500;
        ode_config.sor_pgs_w = 1.3;
        ode_config.sor_pgs_rms_error_tol = 0;
        ode_config.contact_surface_layer = 0.001;
        ode_config.contact_max_correcting_vel = 100;
        ode_config.cfm = 0;
        ode_config.erp = 0.2;
        ode_config.max_contacts = 20;

        gazebo_msgs::SetPhysicsProperties physicsPerperty; // http://docs.ros.org/jade/api/gazebo_msgs/html/srv/SetPhysicsProperties.html

        physicsPerperty.request.time_step = 0.001;  // dt in seconds
        physicsPerperty.request.max_update_rate = 0;  // throttle maximum physics update rate
        physicsPerperty.request.gravity = gravity;
        physicsPerperty.request.ode_config = ode_config;

        while (!setPhysicsClient.call(physicsPerperty) && ros::ok()){
            ROS_ERROR("Failed to call gazebo set physics service, try again ...");
            ros::Duration(0.5).sleep();
        }
    }

    void getGazeboPhysics(){

        getPhysicsClient.waitForExistence();

        gazebo_msgs::GetPhysicsProperties::Request getRequest;
        gazebo_msgs::GetPhysicsProperties::Response getResponse;

        getPhysicsClient.call(getRequest, getResponse);

        cout << "Physics properties:" << endl; 
        cout << getResponse << endl;
    }

};




int main(int argc, char** argv){
    
    ros::init(argc, argv, "jackal_velodyne");
    
    SimulationJackal simJackal;

    ROS_INFO("\033[1;32m---->\033[0m Get or Set Robot Position.");

    return 0;
}
