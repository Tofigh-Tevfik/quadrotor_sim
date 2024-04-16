#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include "quadrotor_sim/QuadrotorInput.h"

namespace gazebo
{
    class QuadrotorPropulsionPlugin : public ModelPlugin
    {
        public:
        // constructor
        // inside constructor we define a ros node to recieve Quadrotor inputs from a ros service
        QuadrotorPropulsionPlugin(): ModelPlugin()
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client");
            rosNode = new ros::NodeHandle("~");
        }
        // destructor
        ~QuadrotorPropulsionPlugin() 
        {
            delete rosNode;
        }

        // callback function for ros service
        // this callback function will set the quadrotor inputs 
        // when the controller send a command
        bool QuadrotorSetInput(quadrotor_sim::QuadrotorInput::Request &req,
                               quadrotor_sim::QuadrotorInput::Response &res)
        {
            // setting the thrust and moment inputs
            thrust.Set(0.0, 0.0, req.T);
            moments.Set(req.Mx, req.My, req.Mz);
            propeller_velocity = req.T * 5.0;
            if (propeller_velocity > 50.0) {
                propeller_velocity = 50.0;
            }
            res.success = true;
            res.message = "Successfully set the inputs";
            return true;
        }

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // storing the model inside a pointer
            this->model = _model;
            // storing the quadrotor link
            this->quadrotor = this->model->GetLink("quadrotor");
            if (!this->quadrotor) {
                gzerr << "quadrotor has not spawned\n";
                return;
            }
            // storing the quadrotor propellers
            // we will add velocity to the propellers just for visualization
            this->propeller_1 = this->model->GetJoint("propeller_1");
            this->propeller_2 = this->model->GetJoint("propeller_2");
            this->propeller_3 = this->model->GetJoint("propeller_3");
            this->propeller_4 = this->model->GetJoint("propeller_4");
            
            // setting the initial thrust to 0
            // this is a force vector acting on the quadrotor body
            // we will only change the 3rd index which is force along the z axis of the quadrotor
            thrust.Set(0.0, 0.0, 0.0);
            // setting the initial moment to 0
            moments.Set(0.0, 0.0, 0.0);
            // initial propeller velocity is 0.0
            propeller_velocity = 0.0;
            // a ros service to recieve inputs from another ros node
            quadrotorInput = rosNode->advertiseService("/quadrotor_input", &QuadrotorPropulsionPlugin::QuadrotorSetInput, this);

            // Listen to the update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&QuadrotorPropulsionPlugin::OnUpdate, this)
            );
            
        }

        // Here in the OnUpdate method
        // we just set apply the thrust and moments 
        // to the quadrotor
        void OnUpdate() {
            // applying the thrust and moments
            this->quadrotor->AddRelativeForce(thrust);
            this->quadrotor->AddRelativeTorque(moments);
            // applying the propeller velocity
            propeller_1->SetVelocity(0, propeller_velocity);
            propeller_2->SetVelocity(0, -propeller_velocity);
            propeller_3->SetVelocity(0, propeller_velocity);
            propeller_4->SetVelocity(0, -propeller_velocity);
        }

        private:
        ros::NodeHandle *rosNode;
        physics::ModelPtr model;
        physics::LinkPtr quadrotor;
        physics::JointPtr propeller_1;
        physics::JointPtr propeller_2;
        physics::JointPtr propeller_3;
        physics::JointPtr propeller_4;
        event::ConnectionPtr updateConnection;

        float propeller_velocity;

        ignition::math::Vector3d thrust;
        ignition::math::Vector3d moments;

        ros::ServiceServer quadrotorInput;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(QuadrotorPropulsionPlugin)
}