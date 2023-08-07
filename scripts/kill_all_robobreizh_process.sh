ps -aux | grep robobreizh_pepper | awk '{print \$2}' | xargs kill -9
