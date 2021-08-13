import rosnode
import os
argv=["/amcl","/map_server","/robot_pose_ekf"]
success, fail = rosnode.kill_nodes(argv)
print("success:",success,"fail:",fail)
# b = os.popen(a,'r',1)