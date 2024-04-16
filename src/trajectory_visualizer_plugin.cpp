#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include "quadrotor_sim/TrajectoryVisualizer.h"
#include "std_srvs/Empty.h"

// we need a gazebo plugin to plot trajectory 
// inside the gazebo

namespace gazebo
{
    class TrajectoryVisualizer : public ModelPlugin
    {   
        public:
        // constructor
        // inside the constructor we initialize a ros node too
        TrajectoryVisualizer(): ModelPlugin() {
            int argc = 0;
            char ** argv = NULL;
            ros::init(argc, argv, "gazebo_trajectory_visualizer");
            rosNode = new ros::NodeHandle("~");
        }
        // Destructor
        ~TrajectoryVisualizer() {
            delete rosNode;
        }
        
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            this->world = _model->GetWorld();
            trajectoryVisualizerService = rosNode->advertiseService("/trajectory/visualizer", &TrajectoryVisualizer::OnNewTrajectory, this);
            clearMarkers = rosNode->advertiseService("/clear_markers", &TrajectoryVisualizer::ClearMarkers, this);
        }
        // method to insert models
        bool OnNewTrajectory(quadrotor_sim::TrajectoryVisualizer::Request &req, quadrotor_sim::TrajectoryVisualizer::Response &res) {
            number_of_points = req.number_of_points;
            for (int i = 0; i < number_of_points; i += 10) {
                // Create a small sphere model at the given point
                std::ostringstream sphereSDF;
                sphereSDF << "<sdf version ='1.6'>"
                            << "<model name ='trajectory_marker_" << i <<"'>"
                            << "<pose>" << req.x[i] << " " << req.y[i] << " " << req.z[i] << " 0 0 0</pose>"
                            << "<link name ='link'>"
                            << "<gravity>false</gravity>"
                            << "<visual name ='visual'>"
                            << "<geometry>"
                            << "<sphere><radius>0.02</radius></sphere>"
                            << "</geometry>"
                            << "<material><ambient>0 0 1 1</ambient></material>"
                            << "</visual>"
                            << "</link>"
                            << "</model>"
                            << "</sdf>";
                this->world->InsertModelString(sphereSDF.str());
            }
            res.message = "Visualization Done";
            res.success = true;
            return true;
        }
        // method to remove the models
        bool ClearMarkers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
            for (int i = 0; i < number_of_points; i += 10) {
                std::ostringstream model_name;
                model_name << "trajectory_marker_" << i;
                this->world->RemoveModel(model_name.str());
            }
            number_of_points = 0;
            return true;
        }

        private:
        physics::WorldPtr world;
        ros::NodeHandle *rosNode;
        ros::ServiceServer trajectoryVisualizerService;
        ros::ServiceServer clearMarkers;
        int number_of_points;
    };

    // Register the plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(TrajectoryVisualizer)
}