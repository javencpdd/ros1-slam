#!/bin/bash
pkill -f '[f]lat2d_stack.launch'
pkill -f '[s]afe_cmd_vel_mux.py'
pkill -f '[s]can_relay.py'
pkill -x move_base
pkill -x map_server
sleep 1
exit 0
