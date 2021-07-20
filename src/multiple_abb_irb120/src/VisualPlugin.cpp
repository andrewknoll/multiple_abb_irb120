#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include "VisualPlugin.hpp"
#include <iostream>
#include <fstream>
#include <string>
using namespace gazebo;


DeformableVisual::DeformableVisual() : VisualPlugin()
{}

void DeformableVisual::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  this->visual = _parent;

}