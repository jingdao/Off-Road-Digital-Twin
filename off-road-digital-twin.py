# Import libraries and utility functions
import open3d as o3d
import numpy as np
import imageio
import matplotlib.pyplot as plt
import sys

# 1. Load the input point cloud using the Open3D library
pcd_object = o3d.io.read_point_cloud(sys.argv[1])
# Convert to a NumPy array for further processing
initial_pcd = np.hstack([pcd_object.points, pcd_object.colors]).astype(np.float32)
print("Point cloud dimensions are: ", initial_pcd.shape)
np.set_printoptions(precision=3, suppress=True)
print("Point cloud coordinates are:")
print(initial_pcd)

# 2. Tree Detection: Point cloud spatial filtering & Euclidean clustering

minZ = 0.796
maxZ = 2.748
pcd_filtered = initial_pcd[(initial_pcd[:, 2] > minZ) & (initial_pcd[:, 2] < maxZ)]
print("Filtered point cloud from %d points to %d points" % (len(initial_pcd), len(pcd_filtered)))

#Use the cluster_dbscan function from Open3D to perform point cloud clustering
cluster_object = o3d.geometry.PointCloud()
cluster_object.points = o3d.utility.Vector3dVector(pcd_filtered[:, :3])
cluster_label = np.array(cluster_object.cluster_dbscan(eps=0.2, min_points=0, print_progress=False))
num_clusters_found = cluster_label.max() + 1
print('Found %d clusters from %d points'%(num_clusters_found, len(pcd_filtered)))

filtered_cluster_label = np.zeros(cluster_label.shape, dtype=int)
num_filtered_clusters = 0

for i in range(num_clusters_found):
    cluster_points = pcd_filtered[cluster_label == i]
    num_cluster_points = len(cluster_points)
    length = cluster_points[:,0].max() - cluster_points[:,0].min()
    width = cluster_points[:,1].max() - cluster_points[:,1].min()
    height = cluster_points[:,2].max() - cluster_points[:,2].min()
    print('Cluster %d: %d points (length=%.2f width=%.2f height=%.2f)' % (i, num_cluster_points, length, width, height))
    if num_cluster_points > 50 and height > 1.3:
        num_filtered_clusters += 1
        filtered_cluster_label[cluster_label == i] = num_filtered_clusters

print('%d clusters remaining after filtering' % num_filtered_clusters)

# refine tree segmentation with circle RANSAC
def fit_circle(points):
    x1 = points[0,0]
    y1 = points[0,1]
    x2 = points[1,0]
    y2 = points[1,1]
    x3 = points[2,0]
    y3 = points[2,1]
    c = (x1-x2)**2 + (y1-y2)**2
    a = (x2-x3)**2 + (y2-y3)**2
    b = (x3-x1)**2 + (y3-y1)**2   
    s = 2*(a*b + b*c + c*a) - (a*a + b*b + c*c)
    if s < 1e-6:
        return None

    px = (a*(b+c-a)*x1 + b*(c+a-b)*x2 + c*(a+b-c)*x3) / s
    py = (a*(b+c-a)*y1 + b*(c+a-b)*y2 + c*(a+b-c)*y3) / s 
    ar = a**0.5
    br = b**0.5
    cr = c**0.5 
    denom  = (ar+br+cr)*(-ar+br+cr)*(ar-br+cr)*(ar+br-cr)
    if denom < 1e-6:
        return None
    r = ar*br*cr / denom**0.5
    return px, py, r

def distance_to_circle(points, px, py, r):
    return np.abs(np.sqrt((points[:, 0] - px)**2 + (points[:, 1] - py)**2) - r)

def circle_ransac(points, inlier_threshold=0.05, num_iterations=10000):
    best_params = None
    max_inliers = 0
    for i in range(num_iterations):
        sample_points = points[np.random.choice(len(points), 3, replace=False)]
        params = fit_circle(sample_points)
        if params is None:
            continue
        px, py, r = params
        D = distance_to_circle(points, px, py, r)
        num_inliers = np.sum(D < inlier_threshold)
        if num_inliers > max_inliers:
            max_inliers = num_inliers
            best_params = (px, py, r)
    px, py, r = best_params
    D = distance_to_circle(points, px, py, r)    
    inlier_mask = D < inlier_threshold
    return px, py, r, inlier_mask

combined_points = []
combined_colors = []
circle_params = []

for cluster_idx in range(1, filtered_cluster_label.max() + 1):
    current_tree = pcd_filtered[filtered_cluster_label==cluster_idx, :3]
    while True:
        px, py, r, inlier_mask = circle_ransac(current_tree[:, :3])
        inliers = current_tree[inlier_mask]
        if len(inliers) < 100 or r > 0.3:
            break
        # prevent overlapping detections
        if len(circle_params) > 0 and (abs(px - circle_params[-1][0]) < circle_params[-1][2] or abs(py - circle_params[-1][1]) < circle_params[-1][2]):
            break
        print("Cluster %d: found %d/%d tree points at (x=%.2f, y=%.2f, r=%.2f)" % (cluster_idx, len(inliers), len(current_tree), px, py, r))
        combined_points.extend(inliers)
        combined_colors.extend([np.random.random(3)] * len(inliers))
        circle_params.append([px, py, r])
        current_tree = current_tree[~inlier_mask]
        if len(current_tree) < 100:
            break

# save txt file with tree centroids and diameter
np.savetxt('results/tree_coordinates.txt', circle_params, fmt="%.3f")

# 3. Ground segmentation: Adaptive RANSAC

minZ = -1.2
maxZ = 0.6
pcd_filtered_ground = initial_pcd[(initial_pcd[:, 2] > minZ) & (initial_pcd[:, 2] < maxZ)]
print("Filtered point cloud from %d points to %d points" % (len(initial_pcd), len(pcd_filtered_ground)))

# Multi-plane RANSAC

grid_resolution = 1.5
minX = pcd_filtered_ground[:, 0].min()
minY = pcd_filtered_ground[:, 1].min()
maxX = pcd_filtered_ground[:, 0].max()
maxY = pcd_filtered_ground[:, 1].max()
discretizedX = np.floor((pcd_filtered_ground[:, 0] - minX) / grid_resolution).astype(int)
discretizedY = np.floor((pcd_filtered_ground[:, 1] - minY) / grid_resolution).astype(int)
numXBins = discretizedX.max() + 1
numYBins = discretizedY.max() + 1

print('Point cloud dimensions: [%.2f - %.2f] x [%.2f - %.2f]' % (minX, maxX, minY, maxY))
print('Discretized into %d x %d bins' % (numXBins, numYBins))

multi_plane_mask = np.zeros(len(pcd_filtered_ground), dtype=bool)

for currentX in range(numXBins):
    for currentY in range(numYBins):
        region_mask = (discretizedX == currentX) & (discretizedY == currentY)
        region_points = pcd_filtered_ground[region_mask]
        if len(region_points) < 3:
            continue
        region_pcd_object = o3d.geometry.PointCloud()
        region_pcd_object.points = o3d.utility.Vector3dVector(region_points[:, 0:3])
        region_plane_params, region_plane_indices = region_pcd_object.segment_plane(0.1, 3, 100)
        print('Found %d/%d plane points in region (%dx%d)' % (len(region_plane_indices), len(region_points), currentX, currentY))
        region_indices = np.nonzero(region_mask)[0]
        multi_plane_mask[region_indices[region_plane_indices]] = True

print("Found %d/%d points on ground plane" % (multi_plane_mask.sum(), len(pcd_filtered_ground)))

# 4. Meshing

# Perform Delaunay triangulation for full point cloud and save it as an OBJ file

from scipy.spatial import Delaunay
plane_points = pcd_filtered_ground[multi_plane_mask]
tri = Delaunay(plane_points[:, :2])
print("Found %d triangles from %d points" % (len(tri.simplices), len(plane_points)))

plane_points_mesh = o3d.geometry.TriangleMesh()
plane_points_mesh.vertices = o3d.utility.Vector3dVector(plane_points[:, 0:3])
plane_points_mesh.triangles = o3d.utility.Vector3iVector(tri.simplices)

o3d.io.write_triangle_mesh('results/terrain_mesh.obj', plane_points_mesh)

# 5. Terrain Texture Mapping

# Load a higher resolution point cloud of the terrain
pcd_object = o3d.io.read_point_cloud(sys.argv[2])
terrain_points = np.hstack([pcd_object.points, pcd_object.colors]).astype(np.float32)
print("Loading terrain point cloud with %d points ..." % len(terrain_points))

# Multi-plane RANSAC

grid_resolution = 1.5
minX = terrain_points[:, 0].min()
minY = terrain_points[:, 1].min()
maxX = terrain_points[:, 0].max()
maxY = terrain_points[:, 1].max()
discretizedX = np.floor((terrain_points[:, 0] - minX) / grid_resolution).astype(int)
discretizedY = np.floor((terrain_points[:, 1] - minY) / grid_resolution).astype(int)
numXBins = discretizedX.max() + 1
numYBins = discretizedY.max() + 1

print('Point cloud dimensions: [%.2f - %.2f] x [%.2f - %.2f]' % (minX, maxX, minY, maxY))
print('Discretized into %d x %d bins' % (numXBins, numYBins))

multi_plane_mask = np.zeros(len(terrain_points), dtype=bool)

for currentX in range(numXBins):
    for currentY in range(numYBins):
        region_mask = (discretizedX == currentX) & (discretizedY == currentY)
        region_points = terrain_points[region_mask]
        if len(region_points) < 3:
            continue
        region_pcd_object = o3d.geometry.PointCloud()
        region_pcd_object.points = o3d.utility.Vector3dVector(region_points[:, 0:3])
        region_plane_params, region_plane_indices = region_pcd_object.segment_plane(0.1, 3, 100)
        print('Found %d/%d plane points in region (%dx%d)' % (len(region_plane_indices), len(region_points), currentX, currentY))
        region_indices = np.nonzero(region_mask)[0]
        multi_plane_mask[region_indices[region_plane_indices]] = True

print("Found %d/%d points on ground plane" % (multi_plane_mask.sum(), len(terrain_points)))

# generate ground texture images
import cv2
plane_points = terrain_points[multi_plane_mask]
minX, minY = plane_points[:, :2].min(axis=0)
maxX, maxY = plane_points[:, :2].max(axis=0)

for grid_resolution in [0.01]:
    raster_width = int(np.ceil((maxX - minX) / grid_resolution)) + 1
    raster_height = int(np.ceil((maxY - minY) / grid_resolution)) + 1
    print('Create raster with %dx%d pixels' % (raster_width, raster_height))
    raster = np.zeros((raster_height, raster_width, 3))

    plane_points_discretized = np.round((plane_points[:, :2] - [minX, minY]) / grid_resolution).astype(int)
    raster[plane_points_discretized[:, 1], plane_points_discretized[:, 0], :] = plane_points[:, 3:6]

    for gaussian_blur_radius in [10]:
        blurred_raster = cv2.blur(raster,(gaussian_blur_radius, gaussian_blur_radius))
        output_filename = 'results/terrain_texture_%.3f_blur_%d.bmp' % (grid_resolution, gaussian_blur_radius)
        imageio.imwrite(output_filename, blurred_raster)
        print('Saved to', output_filename)

