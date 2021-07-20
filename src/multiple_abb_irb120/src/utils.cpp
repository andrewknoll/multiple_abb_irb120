#include "utils.hpp"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

double normalize(ignition::math::Vector3d point) {
    return sqrtf(point.X() * point.X() + point.Y() * point.Y() + point.Z() * point.Z());
}

ignition::math::Vector3d toIgnitionVector3d(const Eigen::Vector3d& v){
    return ignition::math::Vector3d(v(0), v(1), v(2));
}

Eigen::Vector3d toEigenVector3d(const ignition::math::Vector3d& v){
    return Eigen::Vector3d(v.X(), v.Y(), v.Z());
}

gazebo::msgs::Link makeLinkWithSphereShape(gazebo::physics::ModelPtr model, std::string name, double mass, double radius, gazebo::msgs::Pose* pose){
    /*sdf::ElementPtr link = model->GetSDF()->AddElement("link");
    link->SetName(name);

    gazebo::msgs::SphereGeom* sphere = new gazebo::msgs::SphereGeom();
    sphere->set_radius(0.05);

    gazebo::msgs::Geometry* geometry = new gazebo::msgs::Geometry();
    geometry->set_type(gazebo::msgs::Geometry_Type_SPHERE);
    geometry->set_allocated_sphere(sphere);

    //sdf::ElementPtr collision = 
    gazebo::msgs::Collision collision;
    collision.set_name(name + "_collision");
    collision.set_allocated_geometry(geometry);
    collision.set_allocated_pose(pose);

    gazebo::msgs::Geometry* geometryCopy = new gazebo::msgs::Geometry();
    *geometryCopy = *geometry;

    gazebo::msgs::Pose* poseCopy = new gazebo::msgs::Pose();
    *poseCopy = *pose;

    gazebo::msgs::Visual visual;
    visual.set_allocated_geometry(geometry);
    visual.set_allocated_pose(pose);

    link->InsertElement(gazebo::msgs::CollisionToSDF(collision));
    link->InsertElement(gazebo::msgs::VisualToSDF(visual));
*/
    static gazebo::transport::NodePtr node(new gazebo::transport::Node());
    static gazebo::transport::PublisherPtr modelPub = node->Advertise<gazebo::msgs::Model>("~/model/modify");
    gazebo::msgs::Model modelMsg;
    model->FillMsg(modelMsg);
    gazebo::msgs::AddSphereLink(modelMsg, mass, radius);
    gazebo::msgs::Link link = modelMsg.link(modelMsg.link_size() - 1);
    link.set_name(name);
    link.set_allocated_pose(pose);
    std::cout << "El momento de la verdad xd" << std::endl;
    model->ProcessMsg(modelMsg);
    modelPub->Publish(modelMsg);
    std::cout << "Maribel" << std::endl;
    return link;
}