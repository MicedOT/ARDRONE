import pandas as pd

df1= pd.read_csv("center_radius.csv")
df2= pd.read_csv("corresponding_uav_pose.csv")

df1['frame_id'] = df1.index

df_inner = pd.merge(df2, df1, on='frame_id', how='inner')

df3 = df_inner[['x','y','radius']]

df3.to_csv ('corresponding_center_radius.csv', index = False, header=True)
