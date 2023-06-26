# Digital Twin Creation in Off-road Environments

Supplementary material for our paper [Framework for digital twin creation in off-road environments from LiDAR scans](http://doi.org/10.1117/12.2663632) presented at the SPIE Defense and Commercial Sensing conference 2023.

![framework](figures/framework.png?raw=true)

## LiDAR Data

LiDAR-scanned point clouds used in the paper can be downloaded from the following Google Drive links. 

1. [CAVS\_Backyard\_track\_only.ply](https://drive.google.com/file/d/1UZoh22e8g9w7In62pIuFmtEHVhRBQMwp/view?usp=sharing)
2. [CAVS\_Backyard\_terrain\_only\_0.01.ply](https://drive.google.com/file/d/1dGpQTv501qe9y9rQXEH8vN5Hc07ZvNNA/view?usp=sharing)
3. [CAVS\_Backyard\_Husky\_merged.ply](https://drive.google.com/file/d/125uWzAHpp_8jrUDOrXr5GxqukYwXAwk3/view?usp=sharing)
4. [CAVS\_Backyard\_Husky\_track\_only.ply](https://drive.google.com/file/d/17P_8gRbYPxR0wAljyPUOuKEhukx6mf7E/view?usp=sharing)

## Segmentation, Clustering, and Meshing

To run the code for terrain segmentation, tree clustering, and terrain meshing, use the following command:
    
    python off-road-digital-twin.py data/CAVS_Backyard_track_only.ply data/CAVS_Backyard_terrain_only_0.01.ply

## Results

![tree-results](figures/tree_segmentation_results.png?raw=true)

![terrain-results](figures/terrain_segmentation_results.png?raw=true)

## Citing Our Work
We appreciate your support! If you find our code helpful in your research or work, please consider citing our paper.

```bibtext
@inproceedings{chen2023spie,
author = {Jingdao Chen and Mikias Gugssa and Justin Yee and Jun Wang and Christopher Goodin and Athish Ram Das},
title = {{Framework for digital twin creation in off-road environments from LiDAR scans}},
volume = {12529},
booktitle = {Synthetic Data for Artificial Intelligence and Machine Learning: Tools, Techniques, and Applications},
editor = {Christopher L. Howell and Kimberly E. Manser and Raghuveer M. Rao},
organization = {International Society for Optics and Photonics},
publisher = {SPIE},
pages = {125290F},
keywords = {LiDAR, Off-road, Digital twin, Point cloud, Simulator},
year = {2023},
doi = {10.1117/12.2663632},
URL = {https://doi.org/10.1117/12.2663632},
}
```
