#include <ros/ros.h>
#include <Eigen/Dense>
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <tf/tf.h>
#include "quadrotor_sim/QuadrotorInput.h"
#include "quadrotor_sim/FlatOutputs.h"
// state vector
// initially setting the state vector to 0
std::vector<float> x(13, 0.0f);
bool quadrotor_state_recieved = false;

// setting the desired flatoutputs
// initially zero
Eigen::Vector3d rT(0.0, 0.0, 0.0); // position (x, y, z)
Eigen::Vector3d rTdot(0.0, 0.0, 0.0); // velocity(xdot, ydot, zdot)
Eigen::Vector3d rTddot(0.0, 0.0, 0.0); // acceleration(xddot, yddot, zddot)
Eigen::Vector3d adot(0.0, 0.0, 0.0); // jerk
float psi_T = 0.0, psidot_T = 0.0; // yaw and yaw rate

std::vector<float> getControl(std::vector<float>, Eigen::Vector3d, Eigen::Vector3d,
                  Eigen::Vector3d, Eigen::Vector3d, float, float);
void state_callback(const gazebo_msgs::ModelStates::ConstPtr&);
Eigen::Matrix3d quaternionToRotation(tf::Quaternion);
Eigen::Vector3d rot2AxisAng (Eigen::Matrix3d);
bool set_flatoutputs(quadrotor_sim::FlatOutputs::Request &,
                     quadrotor_sim::FlatOutputs::Response &);
Eigen::Vector4d rot2quat(Eigen::Matrix3d);
int sign(float);

int main(int argc, char ** argv) {
    // initialize the node
    ros::init(argc, argv, "nonlinear_controller");
    ros::NodeHandle rosNode;
    ros::Rate controller_rate(100); // 100 Hz
    // we need a ros subscriber to read the quadrotor state
    // currently we will do this by reading the quadrotor model from gazebo
    ros::Subscriber state_subscriber = rosNode.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, state_callback);
    // ros service server for setting the flatoutputs
    ros::ServiceServer flatoutput_service = rosNode.advertiseService("/set/flatoutputs", set_flatoutputs);

    while (!quadrotor_state_recieved) {
        ros::spinOnce();
        controller_rate.sleep();
    }
    // service client to send input to quadrotor
    ros::ServiceClient controller_command_client = rosNode.serviceClient<quadrotor_sim::QuadrotorInput>("/quadrotor_input");
    quadrotor_sim::QuadrotorInput inputs;

    while(ros::ok()) {
        ros::spinOnce();
        std::vector<float> u = getControl(x, rT, rTdot, rTddot, adot, psi_T, psidot_T);
        inputs.request.T = u[0];
        inputs.request.Mx = u[1];
        inputs.request.My = u[2];
        inputs.request.Mz = u[3];
        controller_command_client.call(inputs);
    }


    ros::spin();
    return 0;
}

void state_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // getting the index of the quadrotor from the ModelStates
    size_t index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), "quadrotor"));
    quadrotor_state_recieved = true;
    // pushing back the positions
    x[0] = msg->pose[index].position.x;
    x[1] = msg->pose[index].position.y;
    x[2] = msg->pose[index].position.z;
    // model states has quaternions
    x[3] = msg->pose[index].orientation.x;
    x[4] = msg->pose[index].orientation.y;
    x[5] = msg->pose[index].orientation.z;
    x[6] = msg->pose[index].orientation.w;
    // pushing back velocities
    x[7] = msg->twist[index].linear.x;
    x[8] = msg->twist[index].linear.y;
    x[9] = msg->twist[index].linear.z;
    // now angular velocities
    x[10] = msg->twist[index].angular.x;
    x[11] = msg->twist[index].angular.y;
    x[12] = msg->twist[index].angular.z;
}

std::vector<float> getControl(std::vector<float>x, Eigen::Vector3d rT, Eigen::Vector3d rTdot,
                  Eigen::Vector3d rTddot, Eigen::Vector3d adot, float psi_T,
                  float psidot_T) {
    // input vector size 4x1
    std::vector<float> u(4, 0.0f);
    // dynamic parameters of the drone
    float m = 1.45; float g = 9.81;
    Eigen::Matrix3d I = Eigen::Matrix3d::Zero(); // inertia tensor
    I(0, 0) = 0.0232; I(1, 1) = 0.0232; I(2, 2) = 0.0468;

    // extracting data from state vector
    Eigen::Vector3d r(x[0], x[1], x[2]);
    Eigen::Vector3d rdot(x[7], x[8], x[9]);
    Eigen::Vector3d w(x[10], x[11], x[12]);
    
    // getting the rotation matrix
    tf::Quaternion quaternions(x[3], x[4], x[5], x[6]);
    Eigen::Matrix3d R_w_b = quaternionToRotation(quaternions);
    // transforming the angular velocities to body rates
    w = R_w_b.transpose() * w;
    // position and velocity error
    Eigen::Vector3d ep = r - rT;
    Eigen::Vector3d ev = rdot - rTdot;
    // gains
    Eigen::Matrix3d kp = Eigen::Matrix3d::Identity() * 5.2;
    Eigen::Matrix3d kv = Eigen::Matrix3d::Identity() * 3.5;
    // Desired force
    Eigen::Vector3d F_des = -kp * ep - kv * ev + m * g * Eigen::Vector3d(0, 0, 1) + m * rTddot;
    Eigen::Vector3d z_b = R_w_b * Eigen::Vector3d(0, 0, 1.0);
    u[0] = F_des.dot(z_b);

    // calculate desired body-frame axes
    Eigen::Vector3d z_b_des = F_des.normalized();
    Eigen::Vector3d x_c_des(cos(psi_T), sin(psi_T), 0.0);
    Eigen::Vector3d y_b_des = z_b_des.cross(x_c_des).normalized();
    Eigen::Vector3d x_b_des = y_b_des.cross(z_b_des);
    // desired rotation
    Eigen::Matrix3d R_des;
    R_des.col(0) = x_b_des; R_des.col(1) = y_b_des; R_des.col(2) = z_b_des;
    // rotation error
    Eigen::Matrix3d rot_error = R_w_b.transpose() * R_des;
    Eigen::Vector4d quat = rot2quat(rot_error);
    Eigen::Vector3d er = sign(quat(3)) * Eigen::Vector3d(quat(0), quat(1), quat(2));
    // desired angular velocity
    Eigen::Vector3d h_w = m / u[0] * (adot - z_b_des.dot(adot) * z_b_des);
    float p = -h_w.dot(y_b_des), q = h_w.dot(x_b_des);
    Eigen::Vector3d w_des(p, q, Eigen::Vector3d(0.0, 0.0, psidot_T).dot(z_b_des));
    // angular velocity error
    Eigen::Vector3d ew = w - w_des;
    // angular rotation (kr) and angular velocity (kw) gains
    Eigen::Matrix3d kr = Eigen::Matrix3d::Identity() * 582.2;
    kr(2, 2) = 8.2;
    Eigen::Matrix3d kw = Eigen::Matrix3d::Identity() * 58.2;
    kw(2, 2) = 5.3;
    // calculating the input moments
    Eigen::Vector3d Moments = (w.cross(I*w) - I*(kr * er + kw * ew));
    u[1] = Moments(0);
    u[2] = Moments(1);
    u[3] = Moments(2);

    // adding saturation
    if(u[0] > 50.0) u[0] = 50.0;
    if(u[0] < 0.0) u[0] = 0.0;

    if(u[1] > 5.0) u[1] = 5.0;
    if(u[1] < -5.0) u[1] = -5.0; 

    if(u[2] > 5.0) u[2] = 5.0;
    if(u[2] < -5.0) u[2] = -5.0; 

    if(u[3] > 5.0) u[3] = 5.0;
    if(u[3] < -5.0) u[3] = -5.0; 
    return u;
}
// function to calculate rotation from quaternions
Eigen::Matrix3d quaternionToRotation(tf::Quaternion quat) {
    Eigen::Matrix3d rotationMatrix;
    tf::Matrix3x3 rot(quat);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotationMatrix(i, j) = rot[i][j];
        }
    }
    return rotationMatrix;
}
// function to convert rotation matrix to axis angle
Eigen::Vector3d rot2AxisAng (Eigen::Matrix3d rotationMatrix) {
    tf::Matrix3x3 rot;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rot[i][j] = rotationMatrix(i, j);
        }
    }
    double roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);
    Eigen::Vector3d axisAng(roll, pitch, yaw);
    return axisAng;
}

bool set_flatoutputs(quadrotor_sim::FlatOutputs::Request &req,
                          quadrotor_sim::FlatOutputs::Response &res) {
    // setting the positions
    rT(0) = req.rT[0]; rT(1) = req.rT[1]; rT(2) = req.rT[2];
    // velocities
    rTdot(0) = req.rTdot[0]; rTdot(1) = req.rTdot[1]; rTdot(2) = req.rTdot[2];
    // accelerations
    rTddot(0) = req.rTddot[0]; rTddot(1) = req.rTddot[1]; rTddot(2) = req.rTddot[2];
    // jerk
    adot(0) = req.adot[0]; adot(1) = req.adot[1]; adot(2) = req.adot[2];
    // psi and psidot
    psi_T = req.psi_T; psidot_T = req.psidot_T;

    res.success = true;
    res.message = "Flatoutputs set successfully";
    return true;
}

// rotation matrix to quat
Eigen::Vector4d rot2quat(Eigen::Matrix3d rot) {
    Eigen::Vector4d quat = Eigen::Vector4d::Zero();
    float t = 0;
    if (rot(2, 2) < 0) {
        if (rot(0, 0) > rot(1, 1)) {
            t = 1 + rot(0, 0) - rot(1, 1) - rot(2, 2);
            quat = Eigen::Vector4d(t, rot(0, 1) + rot(1, 0), rot(2, 0) + rot(0, 2), rot(1, 2) - rot(2, 1));
        }
        else {
            t = 1 - rot(0, 0) + rot(1, 1) - rot(2, 2);
            quat = Eigen::Vector4d(rot(0, 1) + rot(1, 0), t, rot(1, 2) + rot(2, 1), rot(2, 0) - rot(0, 2));
        }
    }
    else {
        if (rot(0, 0) < -rot(1, 1)) {
            t = 1 - rot(0, 0) - rot(1, 1) + rot(2, 2);
            quat = Eigen::Vector4d(rot(2, 0) + rot(0, 2), rot(1, 2) + rot(2, 1), t, rot(0, 1) - rot(1, 0));
        }
        else {
            t = 1 + rot(0, 0) + rot(1, 1) + rot(2, 2);
            quat = Eigen::Vector4d(rot(1, 2) - rot(2, 1), rot(2, 0) - rot(0, 2), rot(0, 1) - rot(1, 0), t);
        }
    }
    quat *= 0.5 / sqrt(t);
    return quat;
}
// returns sign of any real number
// returns 1 if 0.0
int sign(float x) {
    if (x >= 0) {
        return 1;
    }
    return -1;
}