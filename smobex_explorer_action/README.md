# SmObEx: Smart Object Exploration

## SmObEx Explorer Action Package

Package containing:

    - action implementation of the exploration methods

## Unknown Space Finding

Package: [Octomap Tools](https://github.com/miguelriemoliveira/octomap_tools)

![volume](../files/impossible_space_to_know.png)

Orange: Unknown Voxels

## Clusters

After knowing which voxels are unknown, we can cluster them (by connection)

![clusters found](../files/clustes_found2.png)

and then find the centroid of those clusters

![clusters centroids](../files/clustes_found_centroids2.png)

## Pose Evaluation

Evaluated pose:

![Pose Evaluated](../files/one_pose_eval_1.png)

Green: Free Voxels

Red: Occupied Voxels

Orange: Unknown Voxels

Blue: Voxels expected to be known with this pose

Red Wireframe: camera's frustum

Gray Lines: Rays used in raycasting