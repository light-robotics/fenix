import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import csv
import json

def polar_to_cartesian(distance, angle, height, roll):
    # Convert angle from degrees to radians
    angle_rad = np.radians(angle)
    roll_rad = np.radians(-roll)
    
    lidar_bias = 40

    # Calculate Cartesian coordinates
    x = -distance * np.cos(angle_rad) * np.cos(roll_rad)
    y = distance * np.sin(angle_rad) * np.cos(roll_rad) + lidar_bias * np.cos(roll_rad)
    z = distance * np.sin(roll_rad) + height + lidar_bias * np.sin(roll_rad)
    
    return x, y, z

# Lidar scan data represented as a list of dictionaries
lidar_scan_data = []
with open ('/fenix/fenix/logs/lidar_data.log', 'r') as f:
    content = f.readlines()

for row in content:
    data = json.loads(row)
    data["height"] = data["height"] * 10
    if data["distance"] > 300 and data["distance"] < 1000:
        print(data)
        lidar_scan_data.append(data)
    #lidar_scan_data.append(json.loads(row))

# Convert lidar scan data to points
points = []
for scan_point in lidar_scan_data:
    x, y, z = polar_to_cartesian(scan_point["distance"], scan_point["angle"],
                                  scan_point["height"], scan_point["roll"])
    points.append((x, y, z))

points_filtered = []
for item in points:
    if item[1] < 220 or item[2] < 20: # !!!!!!!! remove this when horizontal clusterization is fixed
        print(f'Bad point: {item}')
        continue
    points_filtered.append(item)

class Surface:
    # Clustering parameters
    min_samples = 2  # Minimum number of samples in a neighborhood for a point to be considered as a core point
    min_cluster_size = 5  # Minimum number of points in a cluster
    eps = 12.0  # Maximum distance between two samples for them to be considered as in the same Z cluster for surfaces
    surfaces_overlap = 10 # for vertical propogation
    squares_diff = 3 # for vertical propogation

    def __init__(self, surface_label, points):
        self.surface_label = surface_label
        self.points = points
        self.max_x, self.max_y, self.max_z = np.max(points, axis=0)
        self.min_x, self.min_y, self.min_z = np.min(points, axis=0)
        self.min_z = 0
        
        self.base_surface: Surface = None
    
    def __repr__(self):
        if self.base_surface is None:
            return f'[{self.surface_label}]. X: {round(self.min_x, 2)} - {round(self.max_x, 2)}. Y: {round(self.min_y, 2)} - {round(self.max_y, 2)}. Z: {round(self.min_z, 2)} - {round(self.max_z, 2)}. Sq: {round(self.square, 2)}'
        else:
            return f'[{self.surface_label} -> {self.base_surface.surface_label}]. X: {round(self.min_x, 2)} - {round(self.max_x, 2)}. Y: {round(self.min_y, 2)} - {round(self.max_y, 2)}. Z: {round(self.min_z, 2)} - {round(self.max_z, 2)}. Sq: {round(self.square, 2)}'

    def __eq__(self, another):
        return self.surface_label == another.surface_label

    @property
    def square(self):
        return (self.max_x - self.min_x) * (self.max_y - self.min_y)
    
    def propagate_down(self, all_surfaces):
        # Find the closest surface below or to the ground (z=0)
        min_distance = np.inf
        closest_surface = None
        for surface in all_surfaces:
            if surface.min_z >= self.min_z and surface.surface_label != self.surface_label:
                distance = self.min_z - surface.max_z
                if distance < min_distance:
                    min_distance = distance
                    closest_surface = surface
        if min_distance < np.inf:
            if (closest_surface.min_x >= self.min_x - self.surfaces_overlap and 
                    closest_surface.max_x <= self.max_x + self.surfaces_overlap and
                        closest_surface.min_y >= self.min_y - self.surfaces_overlap and 
                            closest_surface.max_y <= self.max_y + self.surfaces_overlap and
                                (1 < abs(closest_surface.square/self.square) < self.squares_diff or
                                 1 < abs(self.square/closest_surface.square) < self.squares_diff)):
                self.base_surface = closest_surface
                self.min_z = closest_surface.max_z if closest_surface.max_z > 0 else 0


    def plot_parallelepipeds(self, ax):
        vertices = np.array([
            [self.min_x, self.min_y, self.min_z],
            [self.max_x, self.min_y, self.min_z],
            [self.max_x, self.max_y, self.min_z],
            [self.min_x, self.max_y, self.min_z],
            [self.min_x, self.min_y, self.max_z],
            [self.max_x, self.min_y, self.max_z],
            [self.max_x, self.max_y, self.max_z],
            [self.min_x, self.max_y, self.max_z]
        ])
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ]
        for edge in edges:
            ax.plot3D(*zip(*vertices[edge]), color='g')
        ax.scatter(self.points[:, 0], self.points[:, 1], self.points[:, 2],
                   label=f'Surface Cluster {self.surface_label}')

def merge_clusters(clusters):
    if not clusters:
        return None

    min_x = min([cluster.min_x for cluster in clusters])
    max_x = max([cluster.max_x for cluster in clusters])
    min_y = min([cluster.min_y for cluster in clusters])
    max_y = max([cluster.max_y for cluster in clusters])
    min_z = min([cluster.min_z for cluster in clusters])
    max_z = max([cluster.max_z for cluster in clusters])

    merged_cluster = Surface('Merged', np.array([[0, 0, 0]]))  # Placeholder for merged cluster
    merged_cluster.min_x = min_x
    merged_cluster.max_x = max_x
    merged_cluster.min_y = min_y
    merged_cluster.max_y = max_y
    merged_cluster.min_z = min_z
    merged_cluster.max_z = max_z

    return merged_cluster

def clusterize_surface(
        data, 
        eps=Surface.eps, 
        min_samples=Surface.min_samples, 
        min_cluster_size=Surface.min_cluster_size
        ) -> list[Surface]:
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    clusters_surface = dbscan.fit_predict(data[:, 2].reshape(-1, 1))

    valid_surface_clusters = []
    for surface_label in np.unique(clusters_surface):
        if np.sum(clusters_surface == surface_label) >= min_cluster_size:
            valid_surface_clusters.append(surface_label)

    surface_clusters = []
    for surface_label in valid_surface_clusters:
        surface_points = data[clusters_surface == surface_label]
        surface = Surface(surface_label, surface_points)
        surface_clusters.append(surface)

    return surface_clusters

def plot_surface_clusters(surface_clusters):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')

    for surface in surface_clusters:
        surface.plot_parallelepipeds(ax)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_zlim(0, 1000)
    ax.set_title('Lidar Data Clustering with Surface Clusters')
    plt.legend()
    plt.show()

def dump_surface_clusters_to_csv(surface_clusters, filename):
    approaching_distance = 350
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        #writer.writerow(['Surface Label', 'Min X', 'Max X', 'Min Y', 'Max Y', 'Min Z', 'Max Z'])
        
        for surface in surface_clusters:
            if surface.max_z < 0:
                continue
            # inverting X axis for real fenix
            # also subtracting approaching distance cuz he comes closer himself
            writer.writerow([#surface.surface_label,
                             #round(surface.min_x, 1), round(surface.max_x, 1),
                             -round(surface.max_x, 1), -round(surface.min_x, 1),
                             round(surface.min_y - approaching_distance, 1), 
                             round(surface.max_y - approaching_distance, 1),
                             round(surface.min_z, 1), round(surface.max_z, 1)])

# for inclusion we remove base_surface
# for propogation we propogate base_surface from cluster_b
def merge_two_clusters(cluster_a: Surface, cluster_b: Surface, inclusion=True):
    min_x = min(cluster_a.min_x, cluster_b.min_x)
    max_x = max(cluster_a.max_x, cluster_b.max_x)
    min_y = min(cluster_a.min_y, cluster_b.min_y)
    max_y = max(cluster_a.max_y, cluster_b.max_y)
    min_z = min(cluster_a.min_z, cluster_b.min_z, cluster_a.max_z, cluster_b.max_z)
    max_z = max(cluster_a.max_z, cluster_b.max_z, cluster_a.min_z, cluster_b.min_z)
    # WTF TODO: propogate down ruins this values, redo

    merged_cluster = Surface(
        str(cluster_a.surface_label) + '.' + str(cluster_b.surface_label),
        np.array([[0, 0, 0]])
        )  # Placeholder for merged cluster
    merged_cluster.min_x = min_x
    merged_cluster.max_x = max_x
    merged_cluster.min_y = min_y
    merged_cluster.max_y = max_y
    merged_cluster.min_z = min_z
    merged_cluster.max_z = max_z
    if not inclusion:
        merged_cluster.base_surface = cluster_b.base_surface

    return merged_cluster

def surface_included(surface_a: Surface, surface_b: Surface):
    if surface_a.min_x >= surface_b.min_x - Surface.surfaces_overlap and \
       surface_a.max_x <= surface_b.max_x + Surface.surfaces_overlap and \
       surface_a.min_y >= surface_b.min_y - Surface.surfaces_overlap and \
       surface_a.max_y <= surface_b.max_y + Surface.surfaces_overlap and \
       surface_a.min_z >= surface_b.min_z - Surface.surfaces_overlap and \
       surface_a.max_z <= surface_b.max_z + Surface.surfaces_overlap:
        return True

def include_all_surfaces(surfaces: list[Surface]):
    included_surfaces = surfaces.copy()
    for i, surface_a in enumerate(surfaces):
        for j, surface_b in enumerate(surfaces):
            if j >= i:
                continue
            #if surface_a == surface_b:
            #    continue
            if (surface_included(surface_a, surface_b) and surface_included(surface_b, surface_a)) \
                or surface_included(surface_a, surface_b) \
                    or surface_included(surface_b, surface_a):
                merged_surface = merge_two_clusters(surface_a, surface_b)
                included_surfaces.append(merged_surface)
                if surface_a in included_surfaces:
                    included_surfaces.remove(surface_a)
                if surface_b in included_surfaces:
                    included_surfaces.remove(surface_b)
                break
    return included_surfaces

def multiple_inclusions(surfaces: list[Surface]):
    for i in range(len(surfaces)):
        surfaces = include_all_surfaces(surfaces)
        print(f'Iteration {i+1}:\n{[x.surface_label for x in surfaces]}\n\n')
    return surfaces

def merge_all_clusters(surface_clusters):
    merged_clusters = surface_clusters.copy()  # Start with a copy of the original clusters

    # Merge clusters recursively
    for _ in range(round(len(merged_clusters)/2 + 1)):
        for cluster in merged_clusters:
            base_surface = cluster.base_surface
            if base_surface in merged_clusters:
                merged_cluster = merge_two_clusters(cluster, base_surface)
                merged_clusters.append(merged_cluster)
                merged_clusters.remove(cluster)
                merged_clusters.remove(base_surface)
                break

    return merged_clusters


surface_clusters = clusterize_surface(np.array(points_filtered))

for surface in surface_clusters:
    surface.propagate_down(surface_clusters)
    print(surface)
    print(surface.base_surface)
    print('\n')


included = multiple_inclusions(surface_clusters)
#merged = merge_all_clusters(surface_clusters)
for cl in included:
    print(cl)

dump_surface_clusters_to_csv(included, '/fenix/fenix/wrk/obstacles.csv')
#plot_surface_clusters(included)