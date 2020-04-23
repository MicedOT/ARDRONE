import pandas as pd
import numpy as np


"""
fields = ['rosbagTimestamp','secs','nsecs']

#df1 = pd.read_csv('_slash_ardrone_slash_bottom_slash_image_raw.csv', skipinitialspace=False, usecols=fields)

chunksize = 10 ** 2
columns=['rosbagTimestamp','secs','nsecs']
df1 = pd.DataFrame(columns=columns)
for chunk in pd.read_csv("_slash_ardrone_slash_bottom_slash_image_raw.csv", chunksize=chunksize):
    dft=chunk[['rosbagTimestamp','secs','nsecs']]

    df1=df1.append(dft)
    print("that")
df1.to_csv ('export_dataframe.csv', index = False, header=True)
#df1= pd.read_csv("_slash_ardrone_slash_bottom_slash_image_raw.csv",low_memory=False)

"""
df1= pd.read_csv("export_dataframe.csv")

df1=df1[["rosbagTimestamp"]]

print("done1")
df2= pd.read_csv("_slash_vicon_slash_ARDroneCarre_slash_ARDroneCarre.csv")
print("done2")


df2=df2[['rosbagTimestamp','lx','ly','lz','cx','cy','cz','cw']]
print("done3")
degree_of_accuracy=10000000	#100000000
numero=10000.0
number_of_places=3
df1['rosbagTimestamp']=df1['rosbagTimestamp'].apply(lambda x:int(x/degree_of_accuracy))#.div(10000000)
#df1['rosbagTimestamp']=df1['rosbagTimestamp'].apply(lambda x:float(x/numero))#.div(10.0)
#df1['rosbagTimestamp']=df1['rosbagTimestamp'].round(number_of_places)

df1['frame_id'] = df1.index
#df1['frame_id']=df1['frame_id'].apply(lambda y:int(y-1))
#df1['rosbagTimestamp']=df1['rosbagTimestamp'].apply(np.floor)

#print(df1.shape[0])

df2['rosbagTimestamp']=df2['rosbagTimestamp'].apply(lambda x:int(x/degree_of_accuracy))#.div(10000000)
#df2['rosbagTimestamp']=df2['rosbagTimestamp'].apply(lambda x:float(x/numero))#.div(10.0)
#df2['rosbagTimestamp']=df2['rosbagTimestamp'].round(number_of_places)
#df2['rosbagTimestamp']=df2['rosbagTimestamp'].apply(np.floor)
print(df1)
print(df2)
df_inner = pd.merge(df1, df2, on='rosbagTimestamp', how='inner')
print(df_inner)
df_inner=df_inner.drop_duplicates('rosbagTimestamp',keep='first')
print(df_inner)
print("done4")
#df3 = df_inner[['frame_id','rosbagTimestamp','lx','ly','lz','cx','cy','cz','cw']]
df3 = df_inner[['frame_id','lx','ly','lz','cx','cy','cz','cw']]
df4=df_inner[['frame_id']]
#print(df3.shape)
#print(df3)
#df3.columns = ['frame_id','rosbagTimestamp','position_x','position_y','position_z','orientation_x','orientation_y','orientation_z','orientation_w']
df3.columns = ['frame_id','position_x','position_y','position_z',
                     'orientation_x','orientation_y','orientation_z','orientation_w']
df4.columns = ['frame_id']
df3.to_csv ('corresponding_uav_pose.csv', index = False, header=True)
df4.to_csv ('frames_chosen.csv', index = False, header=True)
