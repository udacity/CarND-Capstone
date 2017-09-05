#!/bin/bash
#
# Script to launch the CarND Unity simulator

USER_PROFILE="profile.tmp"

if [ ! -f "$USER_PROFILE" ];
  then
    echo "What is the full path to your Unity simulator?"
    read unity_path

    # write to the file
    echo "$unity_path" > $USER_PROFILE
  else
    unity_path=$(cat "$USER_PROFILE")
    echo "USER_PROFILE is "$USER_PROFILE
    cat "$USER_PROFILE"
    echo "Unity path is : "$unity_path
fi
pwd
echo "Unity path is : "$unity_path
$unity_path
#~/work/Udacity/SDC_Nanodegree/SystemIntegrationProject/linux_sys_int/system_integration.x86_64
