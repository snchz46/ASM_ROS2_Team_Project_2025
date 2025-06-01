%% 
ros2 topic list
setenv('ROS_DOMAIN_ID','0');
%% 

nodeSub = ros2node('imageSubscriberNode');
disp('🟢 ROS 2 Subscriber Node Created');

imageSub = ros2subscriber(nodeSub, ...
    '/camera/image_raw', ...
    'sensor_msgs/Image', ...
    @imageCallback);
disp('📥 Subscribed to /camera/image_raw');

function imageCallback(msg)
    img = rosReadImage(msg);
    figure(1);
    imshow(img);
    title(['🖼️ Received Image @ ', datestr(now)]);
end
