# Digital Twin Creation in Off-road Environments

Supplementary material for our paper titled "Framework for digital twin creation in off-road environments
from LiDAR scans".

![framework](figures/framework.png?raw=true)

## LiDAR Data

LiDAR-scanned point clouds used in the paper can be downloaded from the following Google Drive links. 

1. [CAVS\_Backyard\_track\_only.ply](https://drive.google.com/file/d/1UZoh22e8g9w7In62pIuFmtEHVhRBQMwp/view?usp=sharing)
2. [CAVS\_Backyard\_terrain\_only\_0.01.ply](https://drive.google.com/file/d/1dGpQTv501qe9y9rQXEH8vN5Hc07ZvNNA/view?usp=sharing)
3. [CAVS\_Backyard\_Husky\_track\_only.ply](https://drive.google.com/file/d/125uWzAHpp_8jrUDOrXr5GxqukYwXAwk3/view?usp=sharing)

## Segmentation, Clustering, and Meshing

To run the code for terrain segmentation, tree clustering, and terrain meshing, use the following command:
    
    python off-road-digital-twin.py data/CAVS_Backyard_track_only.ply data/CAVS_Backyard_terrain_only_0.01.ply

## Results

![tree-results](figures/tree_segmentation_results.png?raw=true)

![terrain-results](figures/terrain_segmentation_results.png?raw=true)


