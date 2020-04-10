import matplotlib.pyplot as plt
import pandas as pd
from sklearn.cluster import KMeans


X=[]
Y=[]

df=pd.read_csv("global_image_coordinates.csv")
number_of_iterations=len(df)

"""
for i in range(number_of_iterations):
  X.append(df.loc[i, "X"])
  Y.append(df.loc[i, "Y"])

plt.scatter(X, Y)
plt.show()

"""
#"""
m = KMeans(4)
m.fit(df[['X','Y']])
df['cl'] = m.labels_
df.plot.scatter('X', 'Y', c='cl', colormap='gist_rainbow')
plt.show()
"""
kmeans = KMeans(n_clusters=4)
kmeans = kmeans.fit(df[['X','Y']])
labels = kmeans.predict(df[['X','Y']])
centroids = kmeans.cluster_centers_
print(centroids) # From sci-kit learn
"""
