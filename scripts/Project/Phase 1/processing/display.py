import matplotlib.pyplot as plt
import pandas as pd


X=[]
Y=[]

df=pd.read_csv("global_coordinates.csv")
number_of_iterations=len(df)


for i in range(number_of_iterations):
  X.append(df.loc[i, "X"])
  Y.append(df.loc[i, "Y"])

plt.scatter(X, Y)
plt.show()


