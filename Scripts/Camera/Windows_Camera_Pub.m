setenv('ROS_DOMAIN_ID','0');                    %Domain ID
webcam_node = ros2node('yolo_webcam_node');     % Node name
pub_webcam = ros2publisher(webcam_node, '/yolo_webcam_image', 'sensor_msgs/Image');     %Topic name
%%

%webcamlist
cam = webcam(1);                    %Selection of option 1
detector = yolov4ObjectDetector('csp-darknet53-coco');
rate = ros2rate(webcam_node, 5);    % Desired rate = 5 Hz
%%
reset(rate);

for i = 1:1000
    img = snapshot(cam);
    [bboxes, scores, labels] = detect(detector, img);
    annotations = string(labels) + ': ' + string(round(scores,2));
    img = insertObjectAnnotation(img, 'rectangle', bboxes, annotations);
    imshow(img);                              
    title(['Frame ', num2str(i), ' Webcam']);    
    drawnow;                                  

    msgwebcam = ros2message('sensor_msgs/Image');
    msgwebcam.height = uint32(size(img, 1));
    msgwebcam.width = uint32(size(img, 2));
    msgwebcam.encoding = 'rgb8';
    msgwebcam.step = uint32(msgwebcam.width * 3);
    msgwebcam.data = reshape(permute(img, [3 2 1]), [], 1);

    send(pub_webcam, msgwebcam);
    %disp(['Frame ', num2str(i), ' camera']);

    waitfor(rate);
end