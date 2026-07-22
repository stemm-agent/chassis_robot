pip install -U dashscope
pip install -U openai

ros2 topic pub /asr  std_msgs/msg/String  {"data: 小车前进1m，然后左转，然后右转，最后导航去五金店，看看你能看到什么"} --once
ros2 topic pub /asr  std_msgs/msg/String  {"data: 小车前进1m，然后左转，然后右转，我想吃水果了，去看看有什么东西."} --once
