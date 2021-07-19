#!/bin/bash
if [ ! $1 ]; then
  echo "Usage: rosrun marble_mapping map_color.sh [ROBOT_NAME] [agent|rough|normal]"
else
  color=3
  if [ "$2" == "agent" ]; then
    color=1
  elif [ "$2" == "rough" ]; then
    color=2
  fi
  rosrun dynamic_reconfigure dynparam set /$1/marble_mapping display_color $color
fi
