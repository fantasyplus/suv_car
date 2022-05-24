#!/bin/bash

sudo systemctl stop xavier_upstart.service
rosrun robot_upstart uninstall xavier_upstart
