import pandas as pd

df1= pd.read_csv("pic_orientation.csv")
df2= pd.read_csv("corresponding_uav_pose.csv")

df1['frame_id'] = df1.index

df_inner = pd.merge(df2, df1, on='frame_id', how='inner')

df3 = df_inner[["id","frame","X","Y","orientation"]]

df3.to_csv ('corresponding_pic_orientation.csv', index = False, header=True)
