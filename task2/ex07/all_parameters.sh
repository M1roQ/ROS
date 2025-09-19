for node in $(ros2 node list); do
    ros2 param dump $node >> parameter_server.txt
done