#!/usr/bin/env bash
# Kept for the old instructions - same as:  bash confluence.sh start [airport|warehouse] [gazebo|headless|lite]
exec bash "$(dirname "$(readlink -f "$0")")/confluence.sh" start "$@"
