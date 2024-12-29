# Test recording from microphone
alias test-record='arecord -d 5 -f S16_LE -c 1 -r 48000 /tmp/test-mic.wav'
alias test-play='aplay /tmp/test-mic.wav'
alias list-sinks='pactl list short sinks'
alias list-sources='pactl list short sources'

### KUKA
alias build_colcon='colcon build --parallel-workers $(getconf _NPROCESSORS_ONLN)'
alias clean_build_colcon='rm -rf build/ install/ log/ && build_colcon --symlink-install'
alias connect_kuka_pc='sshpass -p "admin" ssh -X kuka@192.168.1.14'
alias kuka_reset='ros2 lifecycle set robot_manager cleanup'
alias kuka_start='ros2 lifecycle set robot_manager configure && ros2 lifecycle set robot_manager activate'
alias kuka_deactivate='ros2 lifecycle set robot_manager deactivate'
alias kuka_deploy='ros2 topic pub --once /command_topic std_msgs/msg/String "data: '\''deploy'\''"'
alias kuka_stow='ros2 topic pub --once /command_topic std_msgs/msg/String "data: '\''stow'\''"'
alias kuka_circular_scanning='ros2 topic pub --once /command_topic std_msgs/msg/String "data: '\''circular_scanning'\''"'
alias kuka_laser_hair_removal='ros2 topic pub --once /command_topic std_msgs/msg/String "data: '\''laser_hair_removal'\''"'
alias kuka_get_face_data='ros2 topic pub --once /command_topic std_msgs/msg/String "data: '\''get_face_data'\''"'
alias kuka_cart_example='ros2 topic pub --once /command_topic std_msgs/msg/String "data: '\''cart_example'\''"'
alias kuka_run_no_hw='ros2 launch kuka_arm_services kuka_motion_control.launch.py client_ip:=10.0.0.2 controller_ip:=10.0.0.4 pitch:=-1.57 roll:=3.14 yaw:=0.0 use_fake_hardware:=true'
alias kuka_run_hw='ros2 launch kuka_arm_services kuka_motion_control.launch.py client_ip:=10.0.0.2 controller_ip:=10.0.0.4 pitch:=-1.57 roll:=3.14 yaw:=0.0 use_fake_hardware:=false'
alias kuka_services='ros2 run kuka_arm_services kuka_arm_services'
alias kuka_clients='ros2 launch kuka_arm_clients kuka_arm_clients.launch.py'
alias vision_run='ros2 run vision_ai treatment_planner'
alias kuka_udp_server='ros2 run udp_listener udp_listener'
alias kuka_go_landmark='ros2 topic pub --once /command_topic std_msgs/msg/String "data: '\''go_face_landmark'\''" --qos-reliability reliable'

# Audio System Nodes
alias run_audio_input="ros2 run kuka_assistant audio_input_node --ros-args --params-file \$KUKA_ASSISTANT_CONFIG_DIR/base_params.yaml --params-file \$KUKA_ASSISTANT_CONFIG_DIR/audio_params.yaml"
alias run_audio_processor="ros2 run kuka_assistant audio_processor_node --ros-args --params-file \$KUKA_ASSISTANT_CONFIG_DIR/base_params.yaml --params-file \$KUKA_ASSISTANT_CONFIG_DIR/audio_params.yaml"

# Core System Nodes
alias run_session_manager="ros2 run kuka_assistant session_manager_node --ros-args --params-file \$KUKA_ASSISTANT_CONFIG_DIR/base_params.yaml --params-file \$KUKA_ASSISTANT_CONFIG_DIR/session_manager_params.yaml"
alias run_speech_detector="ros2 run kuka_assistant speech_event_detector_node --ros-args --params-file \$KUKA_ASSISTANT_CONFIG_DIR/base_params.yaml --params-file \$KUKA_ASSISTANT_CONFIG_DIR/audio_params.yaml"
alias run_conversation="ros2 run kuka_assistant conversation_manager_node"

# Processing Nodes
alias run_transcriber="ros2 run kuka_assistant voice_transcriber_node --ros-args --params-file \$KUKA_ASSISTANT_CONFIG_DIR/base_params.yaml --params-file \$KUKA_ASSISTANT_CONFIG_DIR/voice_transcriber_params.yaml"
alias run_interpreter="ros2 run kuka_assistant text_interpreter_node --ros-args --params-file \$KUKA_ASSISTANT_CONFIG_DIR/text_interpreter_params.yaml"
alias run_tts="ros2 run kuka_assistant text_to_speech_node --ros-args --params-file \$KUKA_ASSISTANT_CONFIG_DIR/base_params.yaml --params-file \$KUKA_ASSISTANT_CONFIG_DIR/text_to_speech_params.yaml"

# Monitoring
alias run_monitor="ros2 run kuka_assistant monitoring_node --ros-args --params-file \$KUKA_ASSISTANT_CONFIG_DIR/base_params.yaml --params-file \$KUKA_ASSISTANT_CONFIG_DIR/monitoring_params.yaml" 