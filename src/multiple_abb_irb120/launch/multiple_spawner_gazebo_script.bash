#!/bin/bash
robots="$1"
dir="$(cd "$(dirname "$0")" && pwd)"
reg_ex="^[0-9]+$"
if [[ $robots =~ $reg_ex ]] ; then
    echo "Spawning $robots robots..."
else
    echo -e "\033[33mWARNING: You have not introduced a valid number. Defaulting to 2...\033[0m"
    robots=2
    echo "Usage: $(basename $0) <number of robots>"
fi

source "$dir/../../../devel/setup.bash"
temp_dir=$(mktemp -d "multiple_robots_ros_packageXXXX")
temp_file=$(mktemp "--tmpdir=$temp_dir" "gazebo_remapper_file_XXXXXX.launch")
i=0

echo "<launch>" > $temp_file

while [ $i -lt $robots ]
do
    i=$((i+1))
    echo "  <remap from=\"/robot$i/arm_controller/follow_joint_trajectory\" to=\"/robot$i/joint_trajectory_action\" />" >> $temp_file
    echo "  <remap from=\"/robot$i/arm_controller/state\" to=\"/robot$i/feedback_states\" />" >> $temp_file
    echo "  <remap from=\"/robot$i/arm_controller/command\" to=\"/robot$i/joint_path_command\"/>" >> $temp_file
done

echo "  <include file=\"\$(find multiple_abb_irb120)/launch/setup_gazebo.launch\">" >> $temp_file
echo "    <arg name=\"robots\" value=\"$robots\"/>" >> $temp_file
echo "  </include>" >> $temp_file

echo "</launch>" >> $temp_file

roslaunch $temp_file
rm $temp_file
rm -r $temp_dir