while true
do
    time rosservice call /robobreizh/perception_pepper/object_detection_service "{entries_list:[data: 'ALL']}" && time rosservice call robobreizh/perception_pepper/person_features_detection_posture "{entries_list:{ obj:[data: ''], distanceMaximum: 5.0}}"
    sleep 10
done
