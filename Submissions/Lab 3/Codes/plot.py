import matplotlib.pyplot as plt
import pandas as pd


X=[]
Y=[]

df=pd.read_csv("global_coordinates.csv")
number_of_iterations=len(df)

read_file="ziles.txt"
r= open(read_file, "r")
for line in r:
    i=int(line)+1
    X.append(df.loc[i, "X"])
    Y.append(df.loc[i, "Y"])


plt.scatter(X, Y)
plt.show()


