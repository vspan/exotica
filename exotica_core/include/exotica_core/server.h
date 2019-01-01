
#ifndef EXOTICA_CORE_SERVER_H
#define EXOTICA_CORE_SERVER_H

#include <map>
#include <typeinfo>

#include <boost/any.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <exotica_core/tools.h>

namespace exotica
{
class RosNode
{
public:
    RosNode() = delete;
    RosNode(std::shared_ptr<ros::NodeHandle> nh, int numThreads = 2);
    ~RosNode();
    inline ros::NodeHandle &GetNodeHandle() { return *nh_; }
    inline tf::TransformBroadcaster &GetTF() { return tf_; }
protected:
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::AsyncSpinner sp_;
    tf::TransformBroadcaster tf_;
};

// Implementation of EXOTica Server class
class Server : public Uncopyable
{
public:
    /// \brief Get the server
    static std::shared_ptr<Server> Instance()
    {
        if (!singleton_server_) singleton_server_.reset(new Server);
        return singleton_server_;
    }
    virtual ~Server();

    /// \brief Check if a robot model exist
    /// @param path Robot model name
    /// @return True if exist, false otherwise
    bool HasModel(const std::string &path);

    /// \brief Get robot model
    /// @param path Robot model name
    /// @param model Robot model
    void GetModel(std::string path, robot_model::RobotModelPtr &model, std::string urdf = "", std::string srdf = "");

    /// \brief Get robot model
    /// @param path Robot model name
    /// @return robot model
    robot_model::RobotModelConstPtr GetModel(std::string path, std::string urdf = "", std::string srdf = "");

    /// \brief Get the name of ther server
    /// @return Server name
    std::string GetName();

    inline static void InitRos(std::shared_ptr<ros::NodeHandle> nh, int numThreads = 2)
    {
        Instance()->node_.reset(new RosNode(nh, numThreads));
    }

    inline static bool IsRos() { return Instance()->node_ != nullptr; }
    inline static ros::NodeHandle &GetNodeHandle()
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        return Instance()->node_->GetNodeHandle();
    }

    template <typename T>
    static bool GetParam(const std::string &name, T &param)
    {
        return Instance()->GetNodeHandle().getParam(name, param);
    }

    template <typename T>
    static void SetParam(const std::string &name, T &param)
    {
        Instance()->GetNodeHandle().setParam(name, param);
    }

    inline bool static HasParam(const std::string &name)
    {
        if (IsRos())
        {
            return Instance()->GetNodeHandle().hasParam(name);
        }
        else
        {
            return false;
        }
    }

    template <typename T, typename... Args>
    static ros::Publisher Advertise(Args &&... args)
    {
        return Instance()->GetNodeHandle().advertise<T>(std::forward<Args>(args)...);
    }

    template <typename T, typename... Args>
    static ros::Subscriber Subscribe(Args &&... args)
    {
        return Instance()->GetNodeHandle().subscribe<T>(std::forward<Args>(args)...);
    }

    template <typename T>
    static ros::ServiceClient ServiceClient(const std::string &service_name, bool persistent = false)
    {
        return Instance()->GetNodeHandle().serviceClient<T>(service_name, persistent);
    }

    static void SendTransform(const tf::StampedTransform &transform)
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        Instance()->node_->GetTF().sendTransform(transform);
    }

    static void SendTransform(const std::vector<tf::StampedTransform> &transforms)
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        Instance()->node_->GetTF().sendTransform(transforms);
    }

    static void SendTransform(const geometry_msgs::TransformStamped &transform)
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        Instance()->node_->GetTF().sendTransform(transform);
    }

    static void SendTransform(const std::vector<geometry_msgs::TransformStamped> &transforms)
    {
        if (!IsRos()) ThrowPretty("EXOTica server not initialized as ROS node!");
        Instance()->node_->GetTF().sendTransform(transforms);
    }

    static void Destroy();

private:
    Server();
    static std::shared_ptr<Server> singleton_server_;
    ///	\brief	Make sure the singleton does not get copied
    Server(Server const &) = delete;
    void operator=(Server const &) = delete;
    robot_model::RobotModelPtr LoadModel(std::string name, std::string urdf = "", std::string srdf = "");

    /// \brief	The name of this server
    std::string name_;

    std::shared_ptr<RosNode> node_;

    /// \brief Robot model cache
    std::map<std::string, robot_model::RobotModelPtr> robot_models_;
};

typedef std::shared_ptr<Server> ServerPtr;
}

#endif
