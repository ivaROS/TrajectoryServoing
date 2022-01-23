# Image-Based Trajectory Tracking through Unknown Environments without Absolute Positioning
This paper describes a stereo image-based visual servoing system for trajectory tracking by a non-holonomic robot without externally derived pose information nor a known visual map of the environment. It is called trajectory servoing. The critical component is a feature-based, indirect SLAM method to provide a pool of available features with estimated depth, so that they may be propagated forward in time to generate image feature trajectories for visual servoing. Short and long distance experiments show the benefits of trajectory servoing for navigating unknown areas without absolute positioning. Empirically, trajectory servoing is more accurate than pose-based feedback when both rely on the same underlying SLAM system.

[[**Demo Video**]](https://youtu.be/hOzgUqUTOxY), [[**Arxiv Preprint**]]()

<!-- <img src="https://github.com/ivaROS/PotentialGap/blob/main/assets/coverImg.png" width = 55% height = 55%/> -->

## Supplementary materials

- [Manuscript symbols](https://github.com/ivaROS/TrajectoryServoing/blob/master/SuppMat/symbols.md)
- [Manuscript abbreviations](https://github.com/ivaROS/TrajectoryServoing/blob/master/SuppMat/abbreviations.md)
- [Links to main implementation code files](https://github.com/ivaROS/TrajectoryServoing/blob/master/SuppMat/links_to_algorithm_sections.md)
- [Simulation setups]()
- [Real experiment setups]()
- [Trajectory servoing control gains]()
- [Ablation study of feature replenishment threshold ![\tau_{fr}](https://latex.codecogs.com/svg.latex?\tau_{fr})]()
- [Performance evaluation metrics and results]()

# Dependencies and Installation

- ROS (Kinetic Ubuntu 16.04) [Installation Link](http://wiki.ros.org/kinetic/Installation/Ubuntu)

<!-- See NavBench [https://github.com/ivalab/NavBench](https://github.com/ivalab/NavBench) for rosinstall instructions and launching experiments.  -->

<!-- # BibTex Citation
```
@ARTICLE{9513583,
      author={Xu, Ruoyang and Feng, Shiyu and Vela, Patricio},
      journal={IEEE Robotics and Automation Letters},
      title={Potential Gap: A Gap-Informed Reactive Policy for Safe Hierarchical Navigation},
      year={2021},
      volume={},
      number={},
      pages={1-1},
      doi={10.1109/LRA.2021.3104623}
}
```

```
R. Xu, S. Feng and P. Vela, "Potential Gap: A Gap-Informed Reactive Policy for Safe Hierarchical Navigation," in IEEE Robotics and Automation Letters, doi: 10.1109/LRA.2021.3104623.
``` -->

# License
The source code is released under [MIT](https://opensource.org/licenses/MIT) license. 
