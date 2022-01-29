#include <trajectory_servoing/send_online_trajectory.h>

namespace trajectory_servoing_benchmark
{
	OnlineTrajSender::OnlineTrajSender(ros::NodeHandle& nh, ros::NodeHandle& pnh, tf::TransformListener& tf) :
		nh_(nh),
        pnh_(pnh),
        tf_(tf),
        planner_costmap_ros_(NULL),
        bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")
	{
		std::string global_planner;
        pnh_.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
        pnh_.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        pnh_.param("global_costmap/global_frame", global_frame_, std::string("/map"));

        //set up plan triple buffer
        // planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        // latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        // controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

		planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_->pause();

        //initialize the global planner
        try {
            planner_ = bgp_loader_.createInstance(global_planner);
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
        } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }

        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED(name_,"Stopping costmaps initially");
            planner_costmap_ros_->stop();
        }

        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(); //optional parameter: ros::Duration(cache time) (default=10) (though it doesn't seem to accept it!)
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
	}

	OnlineTrajSender::~OnlineTrajSender()
    {
        if(planner_costmap_ros_ != NULL)
            delete planner_costmap_ros_;

        // delete planner_plan_;
        // delete latest_plan_;
        // delete controller_plan_;
        
        planner_.reset();
    }

    bool OnlineTrajSender::onInit()
    {
        // Start actively updating costmaps based on sensor data
        planner_costmap_ros_->start();
        
        std::string traj_topic = "/plan";
        base_frame_id_ = "base_footprint";
        fixed_frame_id_ = "map";
        odom_frame_id_ = "odom";
        
        pnh_.getParam("traj_topic", traj_topic);
        pnh_.getParam("base_frame_id", base_frame_id_);
        pnh_.getParam("fixed_frame_id", fixed_frame_id_);
        pnh_.getParam("odom_frame_id", odom_frame_id_);
        
        pnh_.setParam("traj_topic", traj_topic);
        pnh_.setParam("base_frame_id", base_frame_id_);
        pnh_.setParam("fixed_frame_id", fixed_frame_id_);
        pnh_.setParam("odom_frame_id", odom_frame_id_);
        
        v_des_ = 0.05;
        
        pnh_.getParam("v_des", v_des_);
        
        pnh_.setParam("v_des", v_des_);
        
        int pointsPerUnit = 5;
        int skipPoints = 0;
        bool useEndConditions = false;
        bool useMiddleConditions = false;
        
        pnh_.getParam("pointsPerUnit", pointsPerUnit);
        pnh_.getParam("skipPoints", skipPoints);
        pnh_.getParam("useEndConditions", useEndConditions);
        pnh_.getParam("useMiddleConditions", useMiddleConditions);
        
        pnh_.setParam("pointsPerUnit", pointsPerUnit);
        pnh_.setParam("skipPoints", skipPoints);
        pnh_.setParam("useEndConditions", useEndConditions);
        pnh_.setParam("useMiddleConditions", useMiddleConditions);
        
        service_ = nh_.advertiseService("/send_online_trajectory/make_plan", &OnlineTrajSender::sendNiTraj, this);

        ni_pub_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("/turtlebot_controller/trajectory_controller/desired_trajectory", 1000);
        
        current_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/send_online_trajectory/goal", 1);

        trajectory_publisher_ = nh_.advertise< nav_msgs::Path >("/send_online_trajectory/desired_trajectory", 10);
        
        traj_csi_ = boost::make_shared<path_smoothing::CubicSplineInterpolator>(pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
        
        return true;
    }

    bool OnlineTrajSender::sendNiTraj(trajectory_servoing::OnlineTrajService::Request& req, 
									  trajectory_servoing::OnlineTrajService::Response& res)
    {
    	geometry_msgs::PoseStamped goal = goalToGlobalFrame(req.goal);

    	boost::unique_lock<boost::recursive_mutex> lock(traj_mutex_);
        planner_goal_ = goal;
        lock.unlock();

        current_goal_pub_.publish(goal);

        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED(name_,"Starting up costmaps that were shut down previously");
            planner_costmap_ros_->start();
        }

        lock.lock();
        geometry_msgs::PoseStamped temp_goal = planner_goal_;
        lock.unlock();
        
        //run planner
        // planner_plan_->clear();
        std::vector<geometry_msgs::PoseStamped> planner_plan;

        bool gotPlan = makePlan(temp_goal, planner_plan);

        if(gotPlan)
        {
            ROS_INFO_NAMED(name_,"Got Plan with %zu points!", planner_plan.size());
            //pointer swap the plans under mutex (the controller will pull from latest_plan_)
            // std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

            // planner_plan_ = latest_plan_;
            // latest_plan_ = temp_plan;
            nav_msgs::Path latest_path;
            latest_path.header.stamp = planner_plan[0].header.stamp;
            latest_path.header.frame_id = global_frame_;
            latest_path.poses = planner_plan;

            nav_msgs::Path local_smoothed_traj = generateLocalTraj(latest_path);
            
            pips_trajectory_msgs::trajectory_points ni_traj = generateNITraj(local_smoothed_traj);

            ni_pub_.publish(ni_traj);

            res.ni_traj = ni_traj;
        }
        else
        {
        	pips_trajectory_msgs::trajectory_points ni_traj;
        	res.ni_traj = ni_traj;
        }
    }

    geometry_msgs::PoseStamped OnlineTrajSender::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
        std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
        tf::Stamped<tf::Pose> goal_pose, global_pose;
        poseStampedMsgToTF(goal_pose_msg, goal_pose);

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.stamp_ = ros::Time();

        try{
            tf_.transformPose(global_frame, goal_pose, global_pose);
        }
        catch(tf::TransformException& ex){
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
            return goal_pose_msg;
        }

        geometry_msgs::PoseStamped global_pose_msg;
        tf::poseStampedTFToMsg(global_pose, global_pose_msg);
        return global_pose_msg;
    }

    bool OnlineTrajSender::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));
        
        //make sure to set the plan to be empty initially
        plan.clear();
        
        //since this gets called on handle activate
        if(planner_costmap_ros_ == NULL) {
            ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
            return false;
        }

        //get the starting pose of the robot
        tf::Stamped<tf::Pose> global_pose;
        if(!planner_costmap_ros_->getRobotPose(global_pose)) {
            ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
            return false;
        }

        geometry_msgs::PoseStamped start;
        tf::poseStampedTFToMsg(global_pose, start);

        //if the planner fails or returns a zero length plan, planning failed
        if(!planner_->makePlan(start, goal, plan) || plan.empty()){
            ROS_DEBUG_NAMED(name_,"Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
            return false;
        }

        return true;
    }

    nav_msgs::Path OnlineTrajSender::generateLocalTraj(const nav_msgs::Path& global_path)
    {
        // Transform the plan to local frame and use cubic spline to interpolate.
        nav_msgs::Path local_traj;
        local_traj = tfBuffer_->transform(global_path, base_frame_id_, ros::Time(0), fixed_frame_id_);
        nav_msgs::Path local_smoothed_traj;
        traj_csi_->interpolatePath(local_traj, local_smoothed_traj);
        // local_smoothed_traj.poses.pop_back();
        
        return local_smoothed_traj;
    }

    pips_trajectory_msgs::trajectory_points OnlineTrajSender::generateNITraj(const nav_msgs::Path& input_path)
    {

    	turtlebot_trajectory_functions::Path::Ptr pathtraj = std::make_shared<turtlebot_trajectory_functions::Path>(input_path, v_des_);
        
        double tf = pathtraj->getTF();
        trajectory_generator::traj_params_ptr params = std::make_shared<trajectory_generator::traj_params>();
        params->tf=tf;
        
        double v_max=.5;
        double w_max=4;
        double a_max=.55;
        double w_dot_max=1.78;
        
        turtlebot_trajectory_generator::near_identity ni(1,5,1,.01, v_max,w_max,a_max,w_dot_max);    
        traj_func_type::Ptr nc=std::make_shared<traj_func_type>(ni);
        nc->setTrajFunc(pathtraj);
        
        traj_type_ptr traj = std::make_shared<traj_type>();
        traj->header = input_path.header;
        traj->trajpntr = nc ;
        traj->params = params;
        
        traj_gen_bridge_.generate_trajectory(traj);
        
        pips_trajectory_msgs::trajectory_points trajectory_msg = traj->toMsg ();
        
        nav_msgs::Path::ConstPtr path_msg = traj->toPathMsg();
        trajectory_publisher_.publish(path_msg);
        
        return trajectory_msg;
    }
}

int main(int argc, char **argv)
{
    std::string name= "send_online_trajectory";
    ros::init(argc, argv, name);
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    tf::TransformListener tf(ros::Duration(10));
    trajectory_servoing_benchmark::OnlineTrajSender sender(nh, pnh, tf);
    sender.onInit();

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

	return 0;
}
