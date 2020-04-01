import pandas as pd

#get corresponding attributes based on rosbagtimestamp
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

df1['rosbagTimestamp']=df1['rosbagTimestamp'].apply(lambda x:int(x/10000000))#.div(10000000)

#print(df1.shape[0])

df2['rosbagTimestamp']=df2['rosbagTimestamp'].apply(lambda x:int(x/10000000))#.div(10000000)
print(df1)
print(df2)
df_inner = pd.merge(df1, df2, on='rosbagTimestamp', how='inner')
print(df_inner)
df_inner=df_inner.drop_duplicates('rosbagTimestamp',keep='first')

print("done4")
df3 = df_inner[['lx','ly','lz','cx','cy','cz','cw']]

#print(df3.shape)
#print(df3)
df3.columns = ['position_x','position_y','position_z',
                     'orientation_x','orientation_y','orientation_z','orientation_w']

df3.to_csv ('corresponding_uav_pose.csv', index = False, header=True)
