#!/bin/bash
robots="$1"
option="$2"
dir="$(cd "$(dirname "$0")" && pwd)"
reg_ex="^[0-9]+$"
testing=0
small=0
table=0

if [[ $robots =~ $reg_ex ]] ; then
    echo "Spawning $robots robots..."
else
    echo -e "\033[33mWARNING: You have not introduced a valid number. Defaulting to 2...\033[0m"
    robots=2
    echo "Usage: $(basename $0) <number of robots> [-d | -s | -t]"
    echo "       -d : debug grid physics"
    echo "       -s : small grid"
    echo "	 -t : use a table"
fi

if [ "$#" -gt "1" ] ; then
    if [ "$option" = "-d" ]; then
        testing=1
        echo "Testing... Robots will not be spawned, and a big grid will spawn."
    elif [ "$option" = "-s" ]; then
        small=1
        echo "Small grid applied."
    elif [ "$option" = "-t" ]; then
	table=1
	echo "Table applied."
    else
        echo -e "\033[33mWARNING: You have introduced an unknown option \"$option\". Ignored.\033[0m"
        echo "Usage: $(basename $0) <number of robots> [-d | -s | -t]"
        echo "       -d : testing grid physics"
        echo "       -s : small grid"
        echo "       -t : use a table"
    fi
fi

if [ "$#" -gt "3" ]; then
    echo -e "\033[33mWARNING: You have introduced too many options (more than 2).\033[0m"
    echo "Usage: $(basename $0) <number of robots> [-d | -s]"
    echo "       -d : testing grid physics"
    echo "       -s : small grid"
    echo "	 -t : use a table"
fi

if [ "$testing" -eq "1" ]; then
    roslaunch "$dir/test_grid.launch"
else

    source "$dir/../../../devel/setup.bash"
    temp_dir=$(mktemp -d "${TMPDIR:-/tmp/}multiple_robots_ros_package.XXXXXXXXXXXX")
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
    echo "    <arg name=\"small\" value=\"$small\"/>" >> $temp_file
    echo "    <arg name=\"table\" value=\"$table\"/>" >> $temp_file
    echo "  </include>" >> $temp_file

    echo "</launch>" >> $temp_file

    roslaunch $temp_file
    rm $temp_file
    rm -r $temp_dir
fi
