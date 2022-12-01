import rospy as ro
from trajectory_msgs.msg import JointTrajectory as JT
from trajectory_msgs.msg import JointTrajectoryPoint as JTp

sim_poses = JT()

def call(jt):
    # Setting sim_poses in the globalscope
    global sim_poses
    # Setting the sim_poses to the recieved values
    sim_poses = jt
    
def whisper():
    pub = ro.Publisher('poses', JT, queue_size=10)
    rate = ro.Rate(10) # 10hz publisher rate

    return pub, rate

def listener():
    ro.Subscriber("poses", JT, call)

if __name__ == '__main__':
    # Initialization
    ro.init_node('listener', anonymous=True, disable_signals=True)
    #Poses = [0.0, -90.0, 0.0, 0.0, 0.0, 0.0]
    
    pub  = None
    rate = None

    # Publish of poses
    try:
        pub, rate = whisper()
    except ro.ROSInterruptException:
        pass
    print("(0-0) { HEYO! We Made it! ]")
    # Subscriber to poses
    try:
        listener()
    except ro.ROSInterruptException:
        pass
    
    i = 0
    temp_sim_pose = JTp()
    sim_poses_for_publish = JT()
    print("(0-0) { Hello! ]")
    while not ro.is_shutdown():
        if i < 10 :
            print("(0-0) { Hello! ]")
            temp_sim_pose.positions = [14.9,-62.3, 5.4,0.0,0.0,0.0]
            sim_poses_for_publish.points.append(temp_sim_pose)
            temp_sim_pose = JTp()
            i+=1
        
        pub.publish(sim_poses_for_publish)
        rate.sleep()

        # Dataprocessing after they are recieved
        sim_poses_for_publish = sim_poses
        print ("(0_0) { points: ") 
        for g in sim_poses.points:
            ro.loginfo(g.positions)
        print(i)
        print ("         ]")


        command = input("(0-0) { Press q to shutdown node: ]   ")
        if command == 'q' :
            ro.signal_shutdown("reason_to_shutdown")
            break

    print("(=-=) { zzZZ ]")