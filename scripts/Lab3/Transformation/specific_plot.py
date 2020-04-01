import matplotlib.pyplot as plt
import pandas as pd


X=[]
Y=[]

df=pd.read_csv("global_coordinates.csv")
number_of_iterations=len(df)

read_file="/home/ubuntu16/Desktop/Nudes/edf/Stuff/ziles.txt"
r= open(read_file, "r")
for line in r:
    i=int(line)
    X.append(df.loc[i, "X"])
    Y.append(df.loc[i, "Y"])


plt.scatter(X, Y)
plt.show()


