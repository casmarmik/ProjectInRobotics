import numpy as np


class delay_data:
    def __init__(self):
        self.act_poses   = np.array([]).reshape(0,6)
        self.sent_poses  = np.array([]).reshape(0,6)
        self.time_stamps = np.array([])

    def add_act(self, pose):
        self.act_poses = np.vstack([self.act_poses, pose])

    def add_sent(self, pose):
        self.sent_poses = np.vstack([self.sent_poses, pose])
    
    def add_time(self, time):
        self.time_stamps = np.append(self.time_stamps, time)


    def save(self, f_act="delay_data_file_act_deci9.csv", f_sent="delay_data_file_sent_deci9.csv", f_time="delay_data_file_time_deci9.csv"):
        path = "/home/toasted/Documents/KUKA_systems/project_in_robotics/KUKA_Communication/Robot_Control/src/pose_delay_data/"
        out_act  = open(path + f_act,  "w")
        out_sent = open(path + f_sent, "w")
        out_time = open(path + f_time, "w")
        pose_len = 6
        print(len(self.time_stamps))
        str_act  = ""
        str_sent = ""
        str_time = ""
        for i in range(0, len(self.time_stamps)):
            for j in range(0, pose_len):
                print(i)
                print(j)
                print(" ")
                str_sent += str(self.sent_poses[i][j]) + ","
                str_act  += str(self.act_poses[i][j])  + ","

            str_act  += "\n"
            str_sent += "\n"
        
            str_time += str(self.time_stamps[i]) + "\n"
        
        out_act.write(str_act)
        out_sent.write(str_sent)
        out_time.write(str_time)


        out_act.close()
        out_sent.close()
        out_time.close()

    def text(self):
        print_str  =   ""
        print_str  +=  "act: "  + str(self.act_poses) + "\n" 
        print_str  +=  "sent: " + str(self.sent_poses) + "\n" 
        print_str  +=  "time: " + str(self.time_stamps)
        return print_str

dd = delay_data()
dd.add_act([1.0,0.0,0.0,0.0,0.0,0.0])
dd.add_act([1.0,0.0,0.0,0.0,0.0,0.0])
dd.add_sent([0.0,1.0,0.0,0.0,0.0,0.0])
dd.add_sent([0.0,1.0,0.0,0.0,0.0,0.0])

dd.add_time(0.0)
dd.add_time(0.0)

print(dd.text())

dd.save()