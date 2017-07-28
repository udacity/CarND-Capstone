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
fi

$unity_path
