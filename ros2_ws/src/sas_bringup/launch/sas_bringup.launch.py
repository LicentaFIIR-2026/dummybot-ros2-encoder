#!/usr/bin/env python3
"""
SAS Bringup Master Launch
Orchestreaza stiva semantica dupa ce robot.launch.py si navxplorer.launch.py
ruleaza deja pe terminale separate. Yolo ruleaza pe nav (systemd autostart).
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


ROUTE_SERVER_PARAMS = (
    "/home/saim/dummybot-ros2-encoder/ros2_ws/src/"
    "semantic_localizer/config/route_server_params.yaml"
)


def generate_launch_description():

    # t=0s: semantic_localizer
    semantic_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("semantic_localizer"),
            "/launch/semantic_localizer_launch.py",
        ])
    )

    # t=3s: route_server, fix comanda pe care o dai tu in terminal
    route_server = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[sas_bringup] Pornesc route_server..."),
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "nav2_route", "route_server",
                    "--ros-args",
                    "--params-file", ROUTE_SERVER_PARAMS,
                    "-r", "plan:=route_plan",
                ],
                output="screen",
            ),
        ],
    )

    # t=7s: lifecycle configure /yolo26_detector (remote pe nav)
    yolo_configure = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="[sas_bringup] Lifecycle configure /yolo26_detector..."),
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/yolo26_detector", "configure"],
                output="screen",
            ),
        ],
    )

    # t=10s: lifecycle activate /yolo26_detector
    yolo_activate = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[sas_bringup] Lifecycle activate /yolo26_detector..."),
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/yolo26_detector", "activate"],
                output="screen",
            ),
        ],
    )

    # t=12s: lifecycle configure /route_server
    route_configure = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="[sas_bringup] Lifecycle configure /route_server..."),
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/route_server", "configure"],
                output="screen",
            ),
        ],
    )

    # t=15s: lifecycle activate /route_server
    route_activate = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="[sas_bringup] Lifecycle activate /route_server..."),
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/route_server", "activate"],
                output="screen",
            ),
        ],
    )

    # t=18s: xplorer_mcp_bridge
    mcp_bridge = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg="[sas_bringup] Pornesc xplorer_mcp_bridge..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("xplorer_mcp_bridge"),
                    "/launch/xplorer_mcp_bridge_launch.py",
                ])
            ),
        ],
    )

    return LaunchDescription([
        LogInfo(msg="[sas_bringup] === Pornire stiva SAS ==="),
        semantic_localizer,
        route_server,
        yolo_configure,
        yolo_activate,
        route_configure,
        route_activate,
        mcp_bridge,
    ])