# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# NOTE adjust the path according to your catkin workspace !!!
# path_directory = '/home/wu/VS_IROS_2021/src/trajectory_servoing_benchmark/offline_trajs/shortBenchTraj'
path_directory = '/home/shiyu/meta_slam_ws/src/trajectory_servoing_benchmark/offline_trajs/shortBenchTraj'
# path_slam_config = '/home/wu/VS_IROS_2021/src/trajectory_servoing_benchmark/configs/slam_configs'
path_slam_config = '/home/shiyu/meta_slam_ws/src/trajectory_servoing_benchmark/configs/slam_configs'
result_root_prefix = '/DATA/IVAL/traj_servoing_benchmark/sec_journal_benchmark/'

record_topics = '/odom ' \
            + '/odom_sparse ' \
            + '/ground_truth/state ' \
            + '/desired_path ' \
            + '/turtlebot_controller/trajectory_controller/desired_trajectory ' \
            + '/cmd_vel_mux/input/navi ' \
            + '/cmd_vel_mux/input/navi/timed '

record_extended = '/ORB_SLAM/camera_pose_in_imu ' \
            + '/visual/odom'

SeqNameList = ['straight','turnning1','twoturnning1','TS','sturnning1'];
SeqLengList = [4,4,4,4,4];
SeqNameList = ['twoturnning1'];
SeqLengList = [4];

# low IMU
IMU_Type = 'mpu6000';
# high IMU
# IMU_Type = 'ADIS16448';

Fwd_Vel_List = [0.3] # [1.0]; # 
Number_GF_List = [500] # [60, 80, 100, 120] # [40, 60, 80, 120, 160];

Num_Repeating = 10 # 50 # 10 # 

SleepTime = 3 # 5 # 
# Duration = 30 # 60

controller_list = ['gt','slam']
# controller_list = ['slam']
noise = False
noise_std = 0.05

do_rectify = str('false');
do_vis = str('false');

planned_lin_vel = str('true')

#----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ALERT = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


for ri, num_gf in enumerate(Number_GF_List):

    if noise:
        Experiment_prefix = 'noise_' + str(noise_std) + '/pose'
    else:
        Experiment_prefix = 'no_noise' + '/pose'

    for vn, fv in enumerate(Fwd_Vel_List):
        for sn, sname in enumerate(SeqNameList):

            SeqName = SeqNameList[sn]


            # NOTE adjust the path according to your working environment !!!
            # Result_root = '/mnt/DATA/tmp/ClosedNav/debug/'
            # Result_root = '/home/wu/DATA/IVAL/traj_servoing_benchmark/test/' \
            #   + 'tsrb/orb_' + str(int(num_gf)) + '/' + IMU_Type + '/' + SeqName + '/'
            Result_root = result_root_prefix \
              + 'tsrb/orb_' + str(int(num_gf)) + '/' + IMU_Type + '/' + SeqName + '/'

            Experiment_dir = Result_root + Experiment_prefix
            cmd_mkdir = 'mkdir -p ' + Experiment_dir
            subprocess.call(cmd_mkdir, shell=True)

            for cn, controller in enumerate(controller_list):

                for iteration in range(5, Num_Repeating):
                # for iteration in [6]:
                    
                    print bcolors.ALERT + "====================================================================" + bcolors.ENDC
                    print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName + "; Vel: " + str(fv) + "; Controller: " + controller

                    slam_save_path = Experiment_dir + '/slam'
                    slam_cmd_mkdir = 'mkdir -p ' + slam_save_path
                    subprocess.call(slam_cmd_mkdir, shell=True)
                    
                    if planned_lin_vel == "true":
                        plv = '_plv'
                    else:
                        plv = ''

                    path_track_logging = slam_save_path + '/vel_' + str(fv) + '_controller_' + controller + plv + '_round' + str(iteration + 1)
                    path_map_logging = slam_save_path + '/vel_' + str(fv) + '_controller_' + controller + plv + '_round' + str(iteration + 1) + '_Map'
                    path_logging = Experiment_dir + '/vel_' + str(fv) + '_controller_' + controller + plv + '_round' + str(iteration + 1)
                    # num_good_feature = str(num_gf*3)
                    num_good_feature = str(num_gf)
                    path_type = SeqName
                    velocity_fwd = str(fv)
                    duration = float(SeqLengList[sn]) / float(fv) + SleepTime

                    cmd_reset  = str('../bash_files/reset_robot.sh') 
                    # cmd_reset = str('rosservice call /gazebo/reset_simulation "{}"')
                    if controller is 'slam' or controller is 'trajectory_servoing':
                        cmd_slam   = str('roslaunch ../launch/gazebo_GF_slam_multisense_stereo.launch' \
                            + ' path_slam_config:=' + path_slam_config \
                            + ' num_good_feature:=' + num_good_feature \
                            + ' path_track_logging:=' + path_track_logging \
                            + ' path_map_logging:=' + path_map_logging \
                            + ' do_rectify:=' + do_rectify \
                            + ' do_vis:=' + do_vis)
                    else:
                        cmd_slam = ''

                    if controller is 'slam':
                        cmd_esti   = str('roslaunch ../launch/gazebo_slam_msf_stereo.launch' \
                            + ' imu_type:=' + IMU_Type)
                    else:
                        cmd_esti = ''

                    if controller is 'trajectory_servoing':
                        cmd_ctrl   = str('roslaunch ../launch/controller/' + controller + '_controller.launch' \
                            + ' reg_enabled:=' + reg_enabled \
                            + ' uncert_enabled:=' + uncert_enabled)
                    else:
                        cmd_ctrl   = str('roslaunch ../launch/controller/' + controller + '_controller.launch' \
                            + ' planned_lin_vel:=' + planned_lin_vel)
                    cmd_plan   = str('roslaunch ../launch/trajectory_generation/gazebo_offline_planning_gt.launch' \
                        + ' waypts_yaml_dir:=' + path_directory \
                        + ' path_type:=' + path_type \
                        + ' velocity_fwd:=' + velocity_fwd \
                        + ' duration:=' + str(duration) )

                    if controller is 'slam':
                        cmd_log   = str('roslaunch ../launch/logging/gazebo_logging.launch' \
                            + ' topics:="' + record_topics + ' ' + record_extended + '"' \
                            + ' path_data_logging:=' + path_logging )
                    else:
                        cmd_log   = str('roslaunch ../launch/logging/gazebo_logging.launch' \
                            + ' topics:="' + record_topics + '"' \
                            + ' path_data_logging:=' + path_logging )
                    cmd_trig   = str("../bash_files/button_start_robot.sh") 

                    print bcolors.WARNING + "cmd_reset: \n" + cmd_reset + bcolors.ENDC
                    print bcolors.WARNING + "cmd_slam: \n"  + cmd_slam  + bcolors.ENDC
                    print bcolors.WARNING + "cmd_esti: \n"  + cmd_esti  + bcolors.ENDC
                    print bcolors.WARNING + "cmd_plan: \n"  + cmd_plan  + bcolors.ENDC
                    print bcolors.WARNING + "cmd_ctrl: \n"  + cmd_ctrl  + bcolors.ENDC
                    print bcolors.WARNING + "cmd_log: \n"   + cmd_log  + bcolors.ENDC
                    print bcolors.WARNING + "cmd_trig: \n"  + cmd_trig  + bcolors.ENDC

                    print bcolors.OKGREEN + "Reset simulation" + bcolors.ENDC
                    subprocess.Popen(cmd_reset, shell=True)

                    print bcolors.OKGREEN + "Sleeping for a few secs to reset gazebo" + bcolors.ENDC
                    time.sleep(SleepTime)
                    # time.sleep(60)

                    if controller is 'slam' or controller is 'trajectory_servoing':
                        print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
                        subprocess.Popen(cmd_slam, shell=True)

                    if controller is 'slam':
                        print bcolors.OKGREEN + "Launching State Estimator" + bcolors.ENDC
                        subprocess.Popen(cmd_esti, shell=True)
                        time.sleep(SleepTime)

                    print bcolors.OKGREEN + "Launching Controller" + bcolors.ENDC
                    subprocess.Popen(cmd_ctrl, shell=True)

                    print bcolors.OKGREEN + "Launching Planner" + bcolors.ENDC
                    subprocess.Popen(cmd_plan, shell=True)

                    print bcolors.OKGREEN + "Launching Logger" + bcolors.ENDC
                    subprocess.Popen(cmd_log, shell=True)
                    
                    print bcolors.OKGREEN + "Sleeping for a few secs to stabilize msf" + bcolors.ENDC
                    time.sleep(SleepTime * 3)
                    
                    Duration = duration + SleepTime
                    print bcolors.OKGREEN + "Start simulation with " + str(Duration) + " secs" + bcolors.ENDC
                    # proc_trig = subprocess.call(cmd_trig, shell=True)
                    subprocess.Popen(cmd_trig, shell=True)

                    time.sleep(Duration)

                    print bcolors.OKGREEN + "Finish simulation, kill the process" + bcolors.ENDC
                    subprocess.call('rosnode kill data_logging', shell=True)
                    time.sleep(SleepTime)
                    # subprocess.call('rosnode kill Stereo', shell=True)
                    if controller is 'slam':
                        subprocess.call('rosnode kill visual_slam', shell=True)
                        subprocess.call('rosnode kill visual_robot_publisher', shell=True)
                        # subprocess.call('pkill StereoFeatures', shell=True)
                        subprocess.call('rosnode kill msf_pose_sensor', shell=True)

                    subprocess.call('rosnode kill odom_converter', shell=True)
                    # subprocess.call('rosnode kill visual_robot_publisher', shell=True)
                    subprocess.call('rosnode kill turtlebot_controller', shell=True)
                    subprocess.call('rosnode kill turtlebot_trajectory_testing', shell=True)
                    subprocess.call('rosnode kill odom_reset', shell=True)
                    subprocess.call('pkill rostopic', shell=True)
                    subprocess.call('pkill -f trajectory_controller_node', shell=True)