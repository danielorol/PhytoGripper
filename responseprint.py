import matplotlib.pyplot as plt
file = open("response.txt",'r')
data = file.read()
data = data.split(',')
effort = []
joint_pos = []
des_joint_pos = []
time = []
length = len(data)-1
for i in range(0,length-length%3):
    if i%3 == 0:
        effort.append(data[i])
        time.append(i)
    elif i%3 == 1:
        joint_pos.append(data[i])
    else:
        des_joint_pos.append(data[i])

#plt.plot(time,effort)
plt.plot(time,joint_pos)
plt.plot(time,des_joint_pos)
plt.legend(['Joint Position','Target Position'])
plt.show()