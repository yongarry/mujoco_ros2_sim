#ifndef _MJROS_H
#define _MJROS_H

//Mujoco include
#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include <thread>
#include <mutex>
#include <chrono>

#include <iomanip>

//Ros include
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <vector>
#include "mujoco_ros_msgs/msg/apply_force.hpp"
#include "mujoco_ros_msgs/msg/joint_set.hpp"
#include "mujoco_ros_msgs/msg/sensor_state.hpp"
#include "mujoco_ros_msgs/msg/sim_status.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <GLFW/glfw3.h>

#include <deque>

#ifdef COMPILE_SHAREDMEMORY
#include "shm_msgs.h"
SHMmsgs *mj_shm_;
int shm_msg_id;

#define USE_SHM true
#else
#define USE_SHM false
#endif

//-------------------------------- global -----------------------------------------------
//-----mujoco var-----
// constants
const int maxgeom = 5000;         // preallocated geom array in mjvScene
const double syncmisalign = 0.1;  // maximum time mis-alignment before re-sync
const double refreshfactor = 0.7; // fraction of refresh available for simulation

// model and data
inline mjModel *m = NULL;
inline mjData *d = NULL;
inline char filename[1000] = "";

// abstract visualization
inline mjvScene scn;
inline mjvCamera cam;
inline mjvOption vopt;
inline mjvPerturb pert;
inline mjvFigure figconstraint;
inline mjvFigure figcost;
inline mjvFigure figtimer;
inline mjvFigure figsize;
inline mjvFigure figsensor;

// OpenGL rendering and UI
inline GLFWvidmode vmode;
inline int windowpos[2];
inline int windowsize[2];
inline mjrContext con;
inline GLFWwindow *window = NULL;
inline mjuiState uistate;
inline mjUI ui0, ui1;

inline int key_ui = 0;

inline int com_latency = 0;

// UI settings not contained in MuJoCo structures
struct setting_
{
    // file
    int exitrequest = 0;

    // option
    int spacing = 0;
    int color = 0;
    int font = 0;
    int ui0 = 1;
    int ui1 = 0;
    int help = 0;
    int info = 1;
    int profiler = 0;
    int sensor = 0;
    int fullscreen = 0;
    int vsync = 1;
    int busywait = 1;
    int realtime = 0;
    int debug = 0;
    int testbtn2 = 1;
    int timecheck = 0;
    int controlui = 0;
    int link_info = 0;

    // simulation
    int run = 0;
    int key = 0;
    char key_s[40] = "0";
    int loadrequest = 0;

    // watch
    char field[mjMAXUITEXT] = "qpos";
    int index = 0;

    // physics: need sync
    int disable[mjNDISABLE];
    int enable[mjNENABLE];

    // rendering: need sync
    int camera = 0;
};

inline setting_ settings;

// section ids
enum
{
    // left ui
    SECT_FILE = 0,
    SECT_OPTION,
    SECT_SIMULATION,
    SECT_PHYSICS,
    SECT_RENDERING,
    SECT_GROUP,
    NSECT0,

    // right ui
    SECT_WATCH = 0,
    SECT_JOINT,
    SECT_CONTROL,
    NSECT1
};

// file section of UI
const mjuiDef defFile[] =
    {
        {mjITEM_SECTION, "File", 1, NULL, "AF"},
        {mjITEM_BUTTON, "Save xml", 2, NULL, ""},
        {mjITEM_BUTTON, "Save mjb", 2, NULL, ""},
        {mjITEM_BUTTON, "Print model", 2, NULL, "CM"},
        {mjITEM_BUTTON, "Print data", 2, NULL, "CD"},
        {mjITEM_BUTTON, "Quit", 1, NULL, "CQ"},
        {mjITEM_END}};

// option section of UI
const mjuiDef defOption[] =
    {
        {mjITEM_SECTION, "Option", 1, NULL, "AO"},
        {mjITEM_SELECT, "Spacing", 1, &settings.spacing, "Tight\nWide"},
        {mjITEM_SELECT, "Color", 1, &settings.color, "Default\nOrange\nWhite\nBlack"},
        {mjITEM_SELECT, "Font", 1, &settings.font, "50 %\n100 %\n150 %\n200 %\n250 %\n300 %"},
        {mjITEM_CHECKINT, "Left UI (Tab)", 1, &settings.ui0, " #258"},
        {mjITEM_CHECKINT, "Right UI", 1, &settings.ui1, "S#258"},
        {mjITEM_CHECKINT, "Help", 2, &settings.help, " #290"},
        {mjITEM_CHECKINT, "Info", 2, &settings.info, " #291"},
        {mjITEM_CHECKINT, "Profiler", 2, &settings.profiler, " #292"},
        {mjITEM_CHECKINT, "Sensor", 2, &settings.sensor, " #293"},
#ifdef __APPLE__
        {mjITEM_CHECKINT, "Fullscreen", 0, &settings.fullscreen, " #294"},
#else
        {mjITEM_CHECKINT, "Fullscreen", 1, &settings.fullscreen, " #294"},
#endif
        {mjITEM_CHECKINT, "Vertical Sync", 1, &settings.vsync, " #295"},
        {mjITEM_CHECKINT, "Real Time", 1, &settings.busywait, " #296"},
        {mjITEM_CHECKINT, "Debug", 1, &settings.debug, " #297"},
        {mjITEM_CHECKINT, "Time Check", 1, &settings.timecheck, " #298"},
        {mjITEM_CHECKINT, "Control by UI", 1, &settings.controlui, ""},
        {mjITEM_CHECKINT, "Link info", 1, &settings.link_info, ""},
        {mjITEM_END}};

// simulation section of UI
inline const mjuiDef defSimulation[] =
    {
        {mjITEM_SECTION, "Simulation", 1, NULL, "AS"},
        {mjITEM_RADIO, "", 2, &settings.run, "Pause\nRun"},
        {mjITEM_BUTTON, "Reset", 2, NULL, "C#259"},
        {mjITEM_BUTTON, "Reload", 2, NULL, "CL"},
        {mjITEM_BUTTON, "Align", 2, NULL, "CA"},
        {mjITEM_BUTTON, "Copy pose", 2, NULL, "CC"},
        //{mjITEM_SLIDERINT, "Key", 3, &settings.key, "0 0"},
        {mjITEM_BUTTON, "Key + ", 2, NULL, " #266"},
        {mjITEM_BUTTON, "Key - ", 2, NULL, " #267"},
        {mjITEM_STATIC, "Key", 2, NULL, " 0"},
        {mjITEM_BUTTON, "Latency + ", 2, NULL, ""},
        {mjITEM_BUTTON, "Latency - ", 2, NULL, ""},
        {mjITEM_STATIC, "Latency", 2, NULL, " 0"},
        {mjITEM_BUTTON, "Reset to key", 3, NULL, " #259"},
        {mjITEM_BUTTON, "Set key", 3},
        {mjITEM_END}};

// watch section of UI
const mjuiDef defWatch[] =
    {
        {mjITEM_SECTION, "Watch", 0, NULL, "AW"},
        {mjITEM_EDITTXT, "Field", 2, settings.field, "qpos"},
        {mjITEM_EDITINT, "Index", 2, &settings.index, "1"},
        {mjITEM_STATIC, "Value", 2, NULL, " "},
        {mjITEM_END}};

// help strings
const char help_content[] =
    "Alt mouse button\n"
    "UI right hold\n"
    "UI title double-click\n"
    "Space\n"
    "Esc\n"
    "Right arrow\n"
    "Left arrow\n"
    "Down arrow\n"
    "Up arrow\n"
    "Page Up\n"
    "Double-click\n"
    "Right double-click\n"
    "Ctrl Right double-click\n"
    "Scroll, middle drag\n"
    "Left drag\n"
    "[Shift] right drag\n"
    "Ctrl [Shift] drag\n"
    "Ctrl [Shift] right drag";

const char help_title[] =
    "Swap left-right\n"
    "Show UI shortcuts\n"
    "Expand/collapse all  \n"
    "Pause\n"
    "Free camera\n"
    "Step forward\n"
    "Step back\n"
    "Step forward 100\n"
    "Step back 100\n"
    "Select parent\n"
    "Select\n"
    "Center\n"
    "Track camera\n"
    "Zoom\n"
    "View rotate\n"
    "View translate\n"
    "Object rotate\n"
    "Object translate";

// info strings
inline char info_title[1000];
inline char info_content[1000];

void profilerinit(void);
void profilerupdate(void);
void profilershow(mjrRect rect);
void sensorinit(void);
void sensorupdate(void);
void sensorshow(mjrRect rect);
void infotext(char *title, char *content, double interval);
void printfield(char *str, void *ptr);
void watch(void);
void makephysics(int oldstate);
void makerendering(int oldstate);
void makegroup(int oldstate);
void makejoint(int oldstate);
void makecontrol(int oldstate);
void makesections(void);
void alignscale(void);
void copykey(void);
mjtNum timer(void);
void cleartimers(void);
void updatesettings(void);
void drop(GLFWwindow *window, int count, const char **paths);
void loadmodel(void);
int uiPredicate(int category, void *userdata);
void uiLayout(mjuiState *state);
void uiEvent(mjuiState *state);
void prepare(void);
void render(GLFWwindow *window);
void simulate(void);
void init();
void rosPollEvents();

inline std::mutex mtx;

//---------------ROS Var-----------------------
inline rclcpp::Node::SharedPtr nh = nullptr;
inline rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
inline rclcpp::Publisher<mujoco_ros_msgs::msg::SensorState>::SharedPtr sensor_state_pub;
inline rclcpp::Subscription<mujoco_ros_msgs::msg::JointSet>::SharedPtr joint_set;
inline rclcpp::Subscription<mujoco_ros_msgs::msg::JointSet>::SharedPtr joint_init;
inline rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sim_command_sub;
inline rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sim_command_pub;
inline rclcpp::Publisher<mujoco_ros_msgs::msg::SimStatus>::SharedPtr sim_status_pub;

// apply external force
inline rclcpp::Subscription<mujoco_ros_msgs::msg::ApplyForce>::SharedPtr force_apply_sub;

// std_msgs::msg::Float32MultiArray ext_force_msg_;
inline mujoco_ros_msgs::msg::ApplyForce ext_force_msg_;
inline bool ext_force_applied_ = false;
inline std::vector<float> applied_ext_force_;
inline unsigned int force_appiedd_link_idx_;
inline mjvGeom* arrow;
void arrowshow(mjvGeom* arrow);
void makeArrow(mjvGeom* arrow);
void force_apply_callback(const mujoco_ros_msgs::msg::ApplyForce::SharedPtr msg);

inline mujoco_ros_msgs::msg::SensorState sensor_state_msg_;
inline mujoco_ros_msgs::msg::SimStatus sim_status_msg_;
inline sensor_msgs::msg::JointState joint_state_msg_;
inline mujoco_ros_msgs::msg::JointSet joint_set_msg_;
inline std_msgs::msg::Float32 sim_time;
inline rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sim_time_pub;

inline std::vector<float> command;
inline std::vector<float> command2;

inline int loadmodel_request = 0;

inline bool ros_time_sync_reset;

//reset start time
inline bool ros_sim_started = true;
inline bool controller_reset_check = true;
inline bool controller_init_check = true;
inline bool reset_request = false;

inline bool pause_check = true;

inline bool pub_total_mode = false;

inline bool use_shm = false;

//bool for custom applied force
inline bool custom_ft_applied = false;

inline rclcpp::Duration sim_time_ros{0, 0};
inline rclcpp::Time sim_time_run;

inline rclcpp::Duration sim_time_now_ros{0, 0};

inline rclcpp::Duration ros_sim_runtime{0, 0};
inline rclcpp::Time sync_time_test;

inline std::string ctrlstat = "Missing";

inline std::vector<float> ctrl_command_temp_;

inline std::deque<std::vector<float>> ctrl_cmd_que_;

inline mjtNum *ctrl_command;
inline mjtNum *ctrl_command2;

inline bool cmd_rcv = false;

// user state for pub

inline float com_time;
inline float dif_time;

inline double t_bf = 0;

inline double sim_cons_time = 0;
//void c_pause();
//void c_slowmotion();
void c_reset();

//---------------------callback functions --------------------------------
void jointset_callback(const mujoco_ros_msgs::msg::JointSet::SharedPtr msg);
void sim_command_callback(const std_msgs::msg::String::SharedPtr msg);
void state_publisher_init();
void state_publisher();
void mujoco_ros_connector_init();
void mycontroller(const mjModel *m, mjData *d);

#endif