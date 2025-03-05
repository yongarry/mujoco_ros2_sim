/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include "mjros.h"

// drop file callback
void drop(GLFWwindow *window, int count, const char **paths)
{
    // make sure list is non-empty
    if (count > 0)
    {
        mju_strncpy(filename, paths[0], 1000);
        settings.loadrequest = 1;
        RCLCPP_INFO(nh->get_logger(),"DROP REQUEST");
    }
}

// load mjb or xml model
void loadmodel(void)
{
    // clear request
    settings.loadrequest = 0;

    // make sure filename is not empty
    if (!filename[0])
        return;

    // load and compile
    char error[500] = "";
    mjModel *mnew = 0;
    if (strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".mjb"))
    {
        mnew = mj_loadModel(filename, NULL);
        if (!mnew)
            strcpy(error, "could not load binary model");
    }
    else
    {
        mnew = mj_loadXML(filename, NULL, error, 500);
    }
    if (!mnew)
    {
        printf("%s\n", error);
        return;
    }

    // compiler warning: print and pause
    if (error[0])
    {
        // mj_forward() below will print the warning message
        printf("Model compiled, but simulation warning (paused):\n  %s\n\n",
               error);
        settings.run = 0;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);

    int i = settings.key;
    d->time = m->key_time[i];
    mju_copy(d->qpos, m->key_qpos + i * m->nq, m->nq);
    mju_copy(d->qvel, m->key_qvel + i * m->nv, m->nv);
    mju_copy(d->act, m->key_act + i * m->na, m->na);

    if (m->actuator_biastype[0])
    {
        mju_copy(d->ctrl, m->key_qpos + 7 + i * m->nq, m->nu);
    }

    mj_forward(m, d);

    ros_sim_started = true;
    ctrl_command = mj_stackAlloc(d, (int)m->nu);
    ctrl_command2 = mj_stackAlloc(d, (int)(m->nbody * 6));

    // re-create scene and context
    mjv_makeScene(m, &scn, maxgeom);
    mjr_makeContext(m, &con, 50 * (settings.font + 1));

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    pert.skinselect = -1;

    // align and scale view, update scene
    alignscale();
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to model name
    if (window && m->names)
    {
        char title[200] = "Simulate : ";
        strcat(title, m->names);
        strcat(title, nh->get_namespace());
        glfwSetWindowTitle(window, title);
    }

    // rebuild UI sections
    makesections();

    // full ui update
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);

    updatesettings();
    mujoco_ros_connector_init();
    std::cout << " MODEL LOADED " << std::endl;
}
// run event loop
int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    nh = std::make_shared<rclcpp::Node>("mujoco_ros");  // Use nh instead of node

    // Get parameters
    std::string key_file = nh->declare_parameter<std::string>("license", "mjkey.txt");
    use_shm = nh->declare_parameter<bool>("use_shm", false);

    // Create publishers & subscribers
    sim_command_sub = nh->create_subscription<std_msgs::msg::String>(
        "/mujoco_ros_interface/sim_command_con2sim", 100,
        sim_command_callback);

    sim_command_pub = nh->create_publisher<std_msgs::msg::String>(
        "/mujoco_ros_interface/sim_command_sim2con", 1);

    force_apply_sub = nh->create_subscription<mujoco_ros_msgs::msg::ApplyForce>(
        "/mujoco_ros_interface/apply_force", 100,
        force_apply_callback);

    if (!use_shm)
    {
        pub_total_mode = nh->declare_parameter<bool>("pub_mode", false);
        std::cout << "Name Space: " << nh->get_namespace() << std::endl;

        std::string prefix = "/mujoco_ros_interface";
        std::string joint_set_name = prefix + "/joint_set";
        std::string sim_status_name = prefix + "/sim_status";
        std::string joint_state_name = prefix + "/joint_states";
        std::string sim_time_name = prefix + "/sim_time";
        std::string sensor_state_name = prefix + "/sensor_states";

        joint_set = nh->create_subscription<mujoco_ros_msgs::msg::JointSet>(
            joint_set_name, 100,
            jointset_callback);

        if (pub_total_mode)
        {
            sim_status_pub = nh->create_publisher<mujoco_ros_msgs::msg::SimStatus>(sim_status_name, 1);
        }
        else
        {
            joint_state_pub = nh->create_publisher<sensor_msgs::msg::JointState>(joint_state_name, 1);
            sim_time_pub = nh->create_publisher<std_msgs::msg::Float32>(sim_time_name, 1);
            sensor_state_pub = nh->create_publisher<mujoco_ros_msgs::msg::SensorState>(sensor_state_name, 1);
        }
    }
    else
    {
#ifdef COMPILE_SHAREDMEMORY
        init_shm(shm_msg_key, shm_msg_id, &mj_shm_);
#endif
    }

    // ROS time variables
    sim_time_ros = rclcpp::Duration(0, 0);
    sim_time_run = nh->now();
    sim_time_now_ros = rclcpp::Duration(0, 0);

    // Initialize everything
    init();

    // Load model if parameter is provided
    std::string model_file;
    if (nh->get_parameter("model_file", model_file))
    {
        mju_strncpy(filename, model_file.c_str(), 1000);
        settings.loadrequest = 2;
        RCLCPP_INFO(nh->get_logger(), "Model is at %s", model_file.c_str());
    }

    // start simulation thread
    std::thread simthread(simulate);

    // event loop
    while ((!glfwWindowShouldClose(window) && !settings.exitrequest) && rclcpp::ok())
    {
        // start exclusive access (block simulation thread)
        mtx.lock();
        // load model (not on first pass, to show "loading" label)
        if (settings.loadrequest == 1)
        {
            RCLCPP_INFO(nh->get_logger(), "Load Request");
            loadmodel();
        }
        else if (settings.loadrequest > 1)
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // ros events
        rosPollEvents();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // deactive MuJoCo
    // mj_deactivate();

    // Send termination message
    std_msgs::msg::String pmsg;
    pmsg.data = "terminate";
    sim_command_pub->publish(pmsg);

#ifdef COMPILE_SHAREDMEMORY
        deleteSharedMemory(shm_msg_id, mj_shm_);
#endif
// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
