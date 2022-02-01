"""This program does the machine learning to obtain the clusters and its corresponding centroid. The cluster formation
helps to formulate the UGV route in such a way that the UGV's travel to the centroid point of the cluster is sufficient
enough for the UAVs to cover the mission points around that cluster and such proximity allow UGV to recharge UAVs whenever needed.
"""

import pandas as pd
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import random_mission_points_distrib_3clusters_bounds_one


def clustered_locations(random_seed):
    _locations = random_mission_points_distrib_3clusters_bounds_one.random_locations(random_seed)

    df = pd.DataFrame(_locations, columns=['x', 'y'])

    kmeans = KMeans(n_clusters=2)
    kmeans.fit(df)

    labels = kmeans.predict(df)
    centroids = kmeans.cluster_centers_

    # print(centroids)
    ctrd = [(13200, 13200)]

    for i in range(len(centroids)):
        ctrd.append((centroids[i][0], centroids[i][1]))

    fig = plt.figure()

    colmap = {1: 'r', 2: 'g', 3: 'b', 4: 'k', 5: 'y'}

    colors = map(lambda x: colmap[x+1], labels)
    colors1 = list(colors)
    plt.scatter(df['x'], df['y'], color=colors1, alpha=0.5, edgecolor='k')
    for idx, centroid in enumerate(centroids):
        plt.scatter(*centroid, color=colmap[idx+1])
    plt.xlim(0, 26400)
    plt.ylim(0, 26400)
    return ctrd, _locations
