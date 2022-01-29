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

record_topics = '/visual/odom ' \
            + '/odom ' \
            + '/odom_sparse ' \
            + '/ground_truth/state ' \
            + '/desired_path ' \
            + '/turtlebot_controller/trajectory_controller/desired_trajectory ' \
            + '/ORB_SLAM/camera_pose_in_imu ' \
            + '/cmd_vel_mux/input/navi ' \
            + '/cmd_vel_mux/input/navi/timed'

SeqNameList = ['straight','turnning1','twoturnning1','TS','sturnning1'];
SeqLengList = [4,4,4,4,4];
SeqNameList = ['twoturnning1'];
SeqLengList = [4];
# SeqNameList = ['straight','turnning1','twoturnning1','TS'];
# SeqLengList = [4,4,4,4];

# low IMU
IMU_Type = 'mpu6000';
# high IMU
# IMU_Type = 'ADIS16448';

Fwd_Vel_List = [0.3] # [1.0]; # 
Number_GF_List = [500] # [60, 80, 100, 120] # [40, 60, 80, 120, 160];

Num_Repeating = 10 # 50 # 10 # 

SleepTime = 3 # 5 # 
# Duration = 30 # 60

controller_list = ['trajectory_servoing']

noise = False
noise_m = 0
noise_std = 0.05
sigmaMu = 1
sigmaNu = 1
sigmaD = 0.15

# reg_enabled_list = [str('false'), str('true')]
# uncert_enabled_list = [str('false'), str('true')]
# odom_topic_list = ['/odom', '/visual/odom']
reg_enabled_list = [str('false')]
uncert_enabled_list = [str('false')]
feedforward_enabled_list = [str('true')]
odom_topic_list = ['/visual/odom']

lambda_ablation = False
param_lambda = 7 # short is 7

do_rectify = str('false');
do_vis = str('false');


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
        Experiment_prefix = 'noise_' + str(noise_std) + '/visual'
    else:
        Experiment_prefix = 'no_noise' + '/visual'

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

            for on, odom_topic in enumerate(odom_topic_list):
                for rn, reg_enabled in enumerate(reg_enabled_list):
                    for un, uncert_enabled in enumerate(uncert_enabled_list):
                        for ff, feedforward_enabled in enumerate(feedforward_enabled_list):
                            # for iteration in range(5, Num_Repeating):
                            for iteration in [7]:
                                
                                print bcolors.ALERT + "====================================================================" + bcolors.ENDC
                                print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName \
                                                    + "; Vel: " + str(fv) + "; Odom: " + odom_topic \
                                                    + "; Reg enabled:" + str(reg_enabled) + "; Uncertainty:" + str(uncert_enabled)

                                slam_save_path = Experiment_dir + '/slam'
                                slam_cmd_mkdir = 'mkdir -p ' + slam_save_path
                                subprocess.call(slam_cmd_mkdir, shell=True)

                                path_track_logging = slam_save_path + '/vel_' + str(fv) + '_round' + str(iteration + 1)
                                path_map_logging = slam_save_path + '/vel_' + str(fv) + '_round' + str(iteration + 1) + '_Map'

                                if odom_topic is '/odom':
                                    ot = 'odom'
                                elif odom_topic is '/visual/odom':
                                    ot = 'vodom'

                                if reg_enabled is str('true'):
                                    re = '_re'
                                else:
                                    re = ''

                                if uncert_enabled is str('true'):
                                    ue = '_ue'
                                else:
                                    ue = ''

                                if feedforward_enabled is str('true'):
                                    ff = '_ff'
                                else:
                                    ff = ''

                                if lambda_ablation:
                                    la = '_l' + str(param_lambda)
                                else:
                                    la = ''

                                path_logging = Experiment_dir + '/vel_' + str(fv) + '_' + ot + re + ue + ff + la + '_round' + str(iteration + 1)
                                # num_good_feature = str(num_gf*3)
                                num_good_feature = str(num_gf)
                                path_type = SeqName
                                velocity_fwd = str(fv)
                                duration = float(SeqLengList[sn]) / float(fv) + SleepTime

                                cmd_reset  = str('../bash_files/reset_robot.sh') 
                                # cmd_reset = str('rosservice call /gazebo/reset_simulation "{}"')
                                cmd_slam   = str('roslaunch ../launch/gazebo_GF_slam_multisense_stereo.launch' \
                                    + ' path_slam_config:=' + path_slam_config \
                                    + ' num_good_feature:=' + num_good_feature \
                                    + ' path_track_logging:=' + path_track_logging \
                                    + ' path_map_logging:=' + path_map_logging \
                                    + ' do_rectify:=' + do_rectify \
                                    + ' do_vis:=' + do_vis)

                                if odom_topic is '/visual/odom':
                                    cmd_esti   = str('roslaunch ../launch/gazebo_slam_msf_stereo.launch' \
                                        + ' imu_type:=' + IMU_Type)
                                else:
                                    cmd_esti = ''

                                cmd_ctrl   = str('roslaunch ../launch/controller/' + controller_list[0] + '_controller.launch' \
                                    + ' reg_enabled:=' + reg_enabled \
                                    + ' uncert_enabled:=' + uncert_enabled \
                                    + ' feedforward_enabled:=' + feedforward_enabled \
                                    + ' odom_topic:=' + odom_topic \
                                    + ' sigmaMu:=' + str(sigmaMu) \
                                    + ' sigmaNu:=' + str(sigmaNu) \
                                    + ' sigmaD:=' + str(sigmaD) \
                                    + ' lambda:=' + str(param_lambda))
                                
                                cmd_plan   = str('roslaunch ../launch/trajectory_generation/gazebo_offline_planning_gt.launch' \
                                    + ' waypts_yaml_dir:=' + path_directory \
                                    + ' path_type:=' + path_type \
                                    + ' velocity_fwd:=' + velocity_fwd \
                                    + ' duration:=' + str(duration) )

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

                                print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
                                subprocess.Popen(cmd_slam, shell=True)

                                if odom_topic is '/visual/odom':
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
                                
                                subprocess.call('rosnode kill visual_slam', shell=True)
                                subprocess.call('rosnode kill visual_robot_publisher', shell=True)
                                # subprocess.call('pkill StereoFeatures', shell=True)
                                if odom_topic is '/visual/odom':
                                    subprocess.call('rosnode kill msf_pose_sensor', shell=True)

                                subprocess.call('rosnode kill odom_converter', shell=True)
                                # subprocess.call('rosnode kill visual_robot_publisher', shell=True)
                                subprocess.call('rosnode kill vs', shell=True)
                                subprocess.call('rosnode kill turtlebot_trajectory_testing', shell=True)
                                subprocess.call('rosnode kill odom_reset', shell=True)
                                subprocess.call('pkill rostopic', shell=True)
                                subprocess.call('pkill -f trajectory_controller_node', shell=True)