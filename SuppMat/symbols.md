### Visual Servoing

| Variable | Meaning |
| :------: | :------ |
| ![g_{\mathcal{R}}^{\mathcal{W}}](https://latex.codecogs.com/svg.latex?g_{\mathcal{R}}^{\mathcal{W}}) | Robot pose state relative to world frame |
| ![\dot{g}_{\mathcal{R}}^{\mathcal{W}}](https://latex.codecogs.com/svg.latex?\dot{g}_{\mathcal{R}}^{\mathcal{W}}) | Derivative of ![g_{\mathcal{R}}^{\mathcal{W}}](https://latex.codecogs.com/svg.latex?g_{\mathcal{R}}^{\mathcal{W}}) |
| ![\boldsymbol{u}](https://latex.codecogs.com/svg.latex?\boldsymbol{u}) | Robot control |
| ![\nu](https://latex.codecogs.com/svg.latex?\nu) | Linear velocity along robot heading direction |
| ![\omega](https://latex.codecogs.com/svg.latex?\omega) | Angular velocity of robot |
| ![h_{\mathcal{C}}^{\mathcal{R}}](https://latex.codecogs.com/svg.latex?h_{\mathcal{C}}^{\mathcal{R}}) | Camera pose state relative to robot frame |
| ![h_{\mathcal{C}}^{\mathcal{W}}](https://latex.codecogs.com/svg.latex?h_{\mathcal{C}}^{\mathcal{W}}) | Camera pose state relative to world frame |
| ![\dot{h}_{\mathcal{C}}^{\mathcal{W}}](https://latex.codecogs.com/svg.latex?\dot{h}_{\mathcal{C}}^{\mathcal{W}}) | Derivative of ![h_{\mathcal{C}}^{\mathcal{W}}](https://latex.codecogs.com/svg.latex?h_{\mathcal{C}}^{\mathcal{W}}) |
| ![\boldsymbol{H}](https://latex.codecogs.com/svg.latex?\boldsymbol{H}) | Camera projection matrix |
| ![q^\mathcal{W}](https://latex.codecogs.com/svg.latex?q^\mathcal{W}) | Point in world frame |
| ![q^\mathcal{C}](https://latex.codecogs.com/svg.latex?q^\mathcal{C}) | Point in camera frame |
| ![r](https://latex.codecogs.com/svg.latex?r) | Camera point |
| ![D](https://latex.codecogs.com/svg.latex?D) | Differential Operator |
| ![\boldsymbol{\mathcal{L}}](https://latex.codecogs.com/svg.latex?\boldsymbol{\mathcal{L}}) | Image Jacobian |
| ![q = (q^1, q^2, q^3)](https://latex.codecogs.com/svg.latex?q=(q^1,q^2,q^3)) | Three elements of point vector |
| ![r = (r^1, r^2)](https://latex.codecogs.com/svg.latex?r=(r^1,r^2)) | Two elements of camera point vector |
| ![f](https://latex.codecogs.com/svg.latex?f) | Focal Length |

### Trajectory Servoing

| Variable | Meaning |
| :------: | :------ |
| ![\boldsymbol{S}](https://latex.codecogs.com/svg.latex?\boldsymbol{S}) | A set of image points in the current camera image |
| ![\boldsymbol{Q}](https://latex.codecogs.com/svg.latex?\boldsymbol{Q}) | A set of points in the world frame |
| ![n_F](https://latex.codecogs.com/svg.latex?n_F) | The number of points |
| ![\boldsymbol{S}^*](https://latex.codecogs.com/svg.latex?\boldsymbol{S}^*) | Desired feature set |
| ![g_{\mathcal{R}}^{\mathcal{W,*}}(t)](https://latex.codecogs.com/svg.latex?g_{\mathcal{R}}^{\mathcal{W,*}}(t)) | Poses of the trajectory by forward simulating |
| ![\nu^*(t)](https://latex.codecogs.com/svg.latex?\nu^*(t)) | Desired linear velocities of the given trajectory |
| ![\omega^*(t)](https://latex.codecogs.com/svg.latex?\omega^*(t)) | Desired angular velocities of the given trajectory |
| ![\boldsymbol{u}^*(t)](https://latex.codecogs.com/svg.latex?\boldsymbol{u}^*(t)) | Desired controls |
| ![\boldsymbol{E}](https://latex.codecogs.com/svg.latex?\boldsymbol{E}) | Error between desired feature set and current feature set |
| ![\boldsymbol{e}](https://latex.codecogs.com/svg.latex?\boldsymbol{e}) | Vectorized version of ![\boldsymbol{E}](https://latex.codecogs.com/svg.latex?\boldsymbol{E}) |
| ![\boldsymbol{s}](https://latex.codecogs.com/svg.latex?\boldsymbol{s}) | Vectorized version of ![\boldsymbol{S}](https://latex.codecogs.com/svg.latex?\boldsymbol{S}) |
| ![\boldsymbol{s}^*](https://latex.codecogs.com/svg.latex?\boldsymbol{s}^*) | Vectorized version of ![\boldsymbol{S}^*](https://latex.codecogs.com/svg.latex?\boldsymbol{S}^*) |
| ![\boldsymbol{q}](https://latex.codecogs.com/svg.latex?\boldsymbol{q}) | Vectorized version of ![\boldsymbol{Q}](https://latex.codecogs.com/svg.latex?\boldsymbol{Q}) |
| ![\boldsymbol{L}](https://latex.codecogs.com/svg.latex?\boldsymbol{L}) | Vectorized version of ![\boldsymbol{\mathcal{L}}](https://latex.codecogs.com/svg.latex?\boldsymbol{\mathcal{L}}) |
| ![(\boldsymbol{L}^1,\boldsymbol{L}^2)](https://latex.codecogs.com/svg.latex?(\boldsymbol{L}^1,\boldsymbol{L}^2)) | Two column vectors of ![\boldsymbol{L}](https://latex.codecogs.com/svg.latex?\boldsymbol{L}) |
| ![\lambda](https://latex.codecogs.com/svg.latex?\lambda) | Control gain |
| ![\Delta\boldsymbol{e}](https://latex.codecogs.com/svg.latex?\Delta\boldsymbol{e}) | Mismatch between the true solution and the computed pseudo-inverse solution |

### Long Distance Feature Replenishment

| Variable | Meaning |
| :------: | :------ |
| ![\boldsymbol{S}_i^*(t)\|_{t_{i,s}}^{t_{i,e}}](https://latex.codecogs.com/svg.latex?\boldsymbol{S}_i^*(t)\|_{t_{i,s}}^{t_{i,e}}) | ith feature trajectory starting from ![t_{i,s}](https://latex.codecogs.com/svg.latex?t_{i,s}) and ending at ![t_{i,e}](https://latex.codecogs.com/svg.latex?t_{i,e})|
| ![\tau_{fr}](https://latex.codecogs.com/svg.latex?\tau_{fr}) | Feature replenishment threshold |
| ![g^*(t_{i,e},t)](https://latex.codecogs.com/svg.latex?g^*(t_{i,e},t)) | Transformation between the current robot pose and a future desired pose on the trajectory |

### Simulation and Experiments

| Variable | Meaning |
| :------: | :------ |
| ![\nu_{cmd}](https://latex.codecogs.com/svg.latex?\nu_{cmd}) | Linear velocity controls of pose-based method |
| ![\omega_{cmd}](https://latex.codecogs.com/svg.latex?\omega_{cmd}) | Angular velocity controls of pose-based method |
| ![\tilde{\theta}](https://latex.codecogs.com/svg.latex?\tilde{\theta}) | Pose error of the orientation |
| ![\tilde{y}](https://latex.codecogs.com/svg.latex?\tilde{y}) | Pose error of the y direction |
| ![k_{\theta}](https://latex.codecogs.com/svg.latex?k_{\theta}) | Angular control gain |
| ![k_{y}](https://latex.codecogs.com/svg.latex?k_{y}) | y direction control gain |
