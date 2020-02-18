import pandas as pd
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

def plotting_3D():
    print("")


def plotting_2D():
    print("")



def pandas_merger():


    return



    

df1 = pd.read_csv("vicon_slash_ARDroneCarre_slash_ARDroneCarre.csv") 
df2 = pd.read_csv("desired_positions.csv")
"""
df3=df1.iloc[::2]

df1['cx']=df1['x']
df2['dx']=df2['x']

df1['cy']=df1['y']
df2['dy']=df2['y']


df1['cz']=df1['z']
df2['dz']=df2['z']

df1['secs'] = df1['secs']*100
df2['secs'] = df2['secs']*100

df1['cnsecs']=df1['nsecs']/10000000
df2['cnsecs']=df2['nsecs']/10000000

df1['accumulation'] = df1['secs'] + df1['cnsecs']
df2['accumulation'] = df2['secs'] + df2['cnsecs']

#df1['accumulation'] = df1[['secs', 'cnsecs']].apply(lambda x: ''.join(x), axis=1)
#df2['accumulation'] = df2[['secs', 'cnsecs']].apply(lambda x: ''.join(x), axis=1)

df_merge_col = pd.merge(df1, df2, on='accumulation') 

plt.plot(df_merge_col['accumulation'],df_merge_col['cx'])
plt.plot(df_merge_col['accumulation'],df_merge_col['dx'])
"""

df1['ax']=df1['cx']
df2['ax']=df2['cx']

df1['ay']=df1['cy']
df2['ay']=df2['cy']


df1['az']=df1['cz']
df2['az']=df2['cz']



"""

for index, row in df1.iterrows():
    listitem=[row['cx'], row['cy'],row['cz'], row['cw']]
    euler_angle = euler_from_quaternion(listitem)
    angle_roll_desired = euler_angle[0]
    angle_pitch_desired = euler_angle[1]
    angle_yaw_desired = euler_angle[2]
    df1.at[index,'ax']=angle_roll_desired
    df1.at[index,'ay']=angle_pitch_desired
    df1.at[index,'az']=angle_yaw_desired

for index, row in df2.iterrows():
    listitem=[row['cx'], row['cy'],row['cz'], row['cw']]
    euler_angle = euler_from_quaternion(listitem)
    angle_roll_desired = euler_angle[0]
    angle_pitch_desired = euler_angle[1]
    angle_yaw_desired = euler_angle[2]
    df2.at[index,'ax']=angle_roll_desired
    df2.at[index,'ay']=angle_pitch_desired
    df2.at[index,'az']=angle_yaw_desired


#[df1['ax'],df1['ay'],df1['az']] = df1.apply(lambda row : get([row['cx'], row['cy'],row['cz'], row['cw']]), axis = 1) 
plt.figure(yaw_graph)
plt.title("Yaw v/s Time")
plt.xlabel("Time")
plt.ylabel("Angle (radian)")
plt.plot(df2['secs'],df2['az'], label="Desired Yaw",color="blue" )
plt.plot(df1['secs'],df1['az'], label="Actual Yaw",color="orange")
plt.legend(loc="upper left")

plt.figure(pitch_graph)
plt.title("Roll Angle v/s Time")
plt.xlabel("Time")
plt.ylabel("Angle (radian)")
plt.plot(df1['secs'],df1['ax'], label="Roll",color="orange")
plt.legend(loc="upper left")
#plt.plot(df2['secs'],df2['ay'])

plt.figure(roll_graph)
plt.title("Pitch Angle v/s Time")
plt.xlabel("Time")
plt.ylabel("Angle (radian)")
plt.plot(df1['secs'],df1['ay'], label="Pitch", color="orange")
plt.legend(loc="upper left")
#plt.plot(df2['secs'],df2['ax'])

"""
x_graph=0
y_graph=1
z_graph=2
plt.figure(x_graph)
plt.title("Position in X v/s Time")
plt.xlabel("Time")
plt.ylabel("Position (m)")
plt.plot(df2['secs'],df2['lx'], label="Desired X",color="blue" )
plt.plot(df1['secs'],df1['lx'], label="Actual X",color="orange")
plt.legend(loc="upper left")

plt.figure(y_graph)
plt.title("Position in Y v/s Time")
plt.xlabel("Time")
plt.ylabel("Position (m)")
plt.plot(df2['secs'],df2['ly'], label="Desired Y",color="blue" )
plt.plot(df1['secs'],df1['ly'], label="Actual Y",color="orange")
plt.legend(loc="upper left")

plt.figure(z_graph)
plt.title("Position in Z v/s Time")
plt.xlabel("Time")
plt.ylabel("Position (m)")
plt.plot(df2['secs'],df2['lz'], label="Desired Z",color="blue" )
plt.plot(df1['secs'],df1['lz'], label="Actual Z",color="orange")
plt.legend(loc="upper left")

plt.show()




