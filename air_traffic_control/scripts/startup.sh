
cd /opt/code/air_traffic_control
source /opt/ros/jazzy/setup.bash
export OTEL_EXPORTER_OTLP_ENDPOINT="${OTEL_EXPORTER_OTLP_ENDPOINT:-http://otelcol:4318}"
colcon build
source /opt/code/air_traffic_control/install/setup.bash
rqt --force-discover&
ros2 launch air_traffic_control traffic_manager.launch.py robot_count:=10
