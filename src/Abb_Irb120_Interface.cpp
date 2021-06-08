
AbbIrb120_Interface::AbbIrb120_Interface(std::string node, unsigned int size){
    commander = node.advertise<trajectory_msgs::JointTrajectory>(node, size);
}

AbbIrb120_Interface::sendTrajectory(trajectory_msgs::JointTrajectory<std::vector> t){
    commander.publish(t);
}