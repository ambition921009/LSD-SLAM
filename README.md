# LSD-SLAM: 大尺度直接法单目SLAM

LSD-SLAM是一个新的单目实时SLAM方法。它是全部采用直接法（不使用关键点/特征）并在电脑上实时创建大规模、半稠密地图。更多信息请见(http://vision.in.tum.de/lsdslam)，在那里你可以找到相应的出版物和Youtube视频，以及一些示例输入数据集和生成rosbag或.ply点云的输出项。


### 相关论文

* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13



# 1. Quickstart / Minimal Setup

首先, 安装LSD-SLAM 2.1 或 2.2 的版本，这取决于你的ubuntu/ROS版本。你现在不需要openFabMap。

下载 [Room Example Sequence](http://vmcremers8.informatik.tu-muenchen.de/lsd/LSD_room.bag.zip) 并解压。


Launch the lsd_slam viewer:

		rosrun lsd_slam_viewer viewer

Launch the lsd_slam main ros node:

		rosrun lsd_slam_core live_slam image:=/image_raw camera_info:=/camera_info

Play the sequence:

		rosbag play ~/LSD_room.bag



你应该会看到一个显示颜色编码深度（来自live_slam）的当前关键帧窗口，以及显示3D地图（来自viewer）的窗口。如果由于某种原因初始化失败（即5s后深度图仍然显示错误），则查看深度图并点击“r”来重新初始化。



# 2. 安装
我们在不同的系统配置下测试LSD-SLAM,包括Ubuntu 12.04(Precise) + ROS fuerte、Ubuntu 14.04 (trusty) and ROS indigo、Ubuntu 16.04（）+ROS kinetic。请注意，不支持不带ROS的构建，但ROS仅用于输入和输出，便于移植到其他平台。


## 2.1 ROS fuerte + Ubuntu 12.04
安装系统依赖项:

    sudo apt-get install ros-fuerte-libg2o liblapack-dev libblas-dev freeglut3-dev libqglviewer-qt4-dev libsuitesparse-dev libx11-dev

在你的ROS包路径下, 克隆以下仓库:

    git clone https://github.com/tum-vision/lsd_slam.git lsd_slam

通过输入以下内容编译两个包:

    rosmake lsd_slam




## 2.2 ROS indigo + Ubuntu 14.04
**我们不使用catkin, 然而幸运的是，传统的CMAKE构建仍然可以与ROS Indigo一起使用。**
对此，你需要创建一个 rosbuild 工作空间(如果你还没有), 使用以下命令:

    sudo apt-get install python-rosinstall
    mkdir ~/rosbuild_ws
    cd ~/rosbuild_ws
    rosws init . /opt/ros/indigo
    mkdir package_dir
    rosws set ~/rosbuild_ws/package_dir -t .
    echo "source ~/rosbuild_ws/setup.bash" >> ~/.bashrc
    bash
    cd package_dir

安装系统依赖项:

    sudo apt-get install ros-indigo-libg2o ros-indigo-cv-bridge liblapack-dev libblas-dev freeglut3-dev libqglviewer-dev libsuitesparse-dev libx11-dev

在你的ROS包路径下, 克隆以下仓库:

    git clone https://github.com/tum-vision/lsd_slam.git lsd_slam

通过输入以下内容编译两个包:

    rosmake lsd_slam



## 2.3 ROS Kinect + Ubuntu 16.04 
这一部分是源代码说明中没有的，本人通过测试讲代码改为适合ROS Kinect + Ubuntu 16.04 环境下运行的版本。


## 2.4 用于大规模回环检测的 openFabMap [可选]
如果你想使用 openFABMAP 进行大规模回环检测, 取消 `lsd_slam_core/CMakeLists.txt` 文件中的下列注释:

    #add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
    #include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
    #add_definitions("-DHAVE_FABMAP")
    #set(FABMAP_LIB openFABMAP )

**Note for Ubuntu 14.04:** Ubuntu 14.04打包的 OpenCV不包含 openFabMap（它需要SURF特征点）所需的非自由模块。
你需要获取包含非自由模块的全部版本的OpenCV，最简单的方法是编译自己的版本。 我们建议使用 [2.4.8]版本(https://github.com/Itseez/opencv/releases/tag/2.4.8), 以确保和当前的indigo版本保持兼容。





# 3 使用
LSD-SLAM 分为两个ROS包, `lsd_slam_core` 和 `lsd_slam_viewer`. `lsd_slam_core` 包含全部的SLAM系统, 反之， `lsd_slam_viewer` 是可选的，用于3D可视化.
也请阅读下文的 **General Notes for good results** 。

## 3.1 `lsd_slam_core`
我们提供了两种不同的使用方式, 一种是使用ROS输入/输出的实时操作(`live_slam`), 一种是使用图像文件的数据集`dataset_slam`。

### 3.1.1 使用 `live_slam`
如果你想直接使用摄像头。

    rosrun lsd_slam_core live_slam /image:=<yourstreamtopic> /camera_info:=<yourcamera_infotopic>

使用ROS摄像机信息时，仅使用摄像机中的图像尺寸和`K`矩阵信息，因此必须校正视频畸变。

或者，可以使用指定校准文件。

    rosrun lsd_slam_core live_slam /image:=<yourstreamtopic> _calib:=<calibration_file>

在这种情况下，摄像机信息主题容易被忽略，图像可能也会有徑向畸变。有关校正文件格式的详细信息，请参阅“相机校准”部分。


### 3.1.2  使用 `dataset_slam`

    rosrun lsd_slam_core dataset_slam _files:=<files> _hz:=<hz> _calib:=<calibration_file>

这里, `<files>`可以是包含图像文件（按字母顺序存储）的文件夹，也可以是每行包含一个图像文件的文本文件。`<hz>`是处理图像的帧速率，以及`<calibration_file>`是相机校准文件。 

指定`_hz:=0`以启用顺序跟踪和映射，即确保每帧都正确映射。注意，虽然这通常会产生最佳结果，但它可能比实时操作慢得多。


### 3.1.3 相机校准
LSD-SLAM在针孔相机模型上工作，但是我们提供了在使用之前使图像不失真的选项。你可以在`lsd_slam_core/calib`中找到一些示例校准文件。

#### FOV相机模型的校准文件:

    fx/width fy/height cx/width cy/height d
    in_width in_height
    "crop" / "full" / "none" / "e1 e2 e3 e4 0"
    out_width out_height


这里，第一行中的值是相机的内部参数和径向畸变参数，这些参数是通过PTAM相机校准器获得的, in\_width 和 in\_height是输入图像的尺寸, out\_width out\_height 是所需的未失真图像的尺寸。后者可以自由选择，但建议使用640x480，如第3.1.6节所述。第三行指定图像的扭曲形式，或指定与前四项内部参数相同格式的所需的相机矩阵, 或通过指定"crop"将图像裁剪到仅包含有效像素的最大区域。


#### 预校正图像的校准文件:
这是一个没有径向畸变校正，作为一个特例的ATAN相机模型，但没有计算成本:

    fx/width fy/height cx/width cy/height 0
    width height
    none
    width height


#### OpenCV相机模型的校准文件:

    fx fy cx cy k1 k2 p1 p2
    inputWidth inputHeight
    "crop" / "full" / "none" / "e1 e2 e3 e4 0"
    outputWidth outputHeight


### 3.1.4 可用的热键

- `r`: Do a full reset

- `d / e`: Cycle through debug displays (in particular color-coded variance and color-coded inverse depth).

- `o`: Toggle on screen info display

- `m`: Save current state of the map (depth & variance) as images to `lsd_slam_core/save/`

- `p`: Brute-Force-Try to find new constraints. May improve the map by finding more constraints, but will block mapping for a while. 

- `l`: Manually indicate that tracking is lost: will stop tracking and mapping, and start the re-localizer.



### 3.1.5 参数 (Dynamic Reconfigure)
A number of things can be changed dynamically, using (for ROS fuerte)

    rosrun dynamic_reconfigure reconfigure_gui 

or (for ROS indigo)

    rosrun rqt_reconfigure rqt_reconfigure

Parameters are split into two parts, ones that enable / disable various sorts of debug output in `/LSD_SLAM/Debug`, and ones that affect the actual algorithm, in `/LSD_SLAM`.
Note that debug output options from `/LSD_SLAM/Debug` only work if lsd\_slam\_core is built with debug info, e.g. with `set(ROS_BUILD_TYPE RelWithDebInfo)`. 

* `minUseGrad`: [double] Minimal absolute image gradient for a pixel to be used at all. Increase if your camera has large image noise, decrease if you have low image-noise and want to also exploit small gradients.
* `cameraPixelNoise`: [double] Image intensity noise used for e.g. tracking weight calculation. Should be set larger than the actual sensor-noise, to also account for noise originating from discretization / linear interpolation.
* `KFUsageWeight`: [double] Determines how often keyframes are taken, depending on the overlap to the current keyframe. Larger -> more keyframes.
* `KFDistWeight`: [double] Determines how often keyframes are taken, depending on the distance to the current Keyframe. Larger -> more keyframes.
* `doSLAM`: [bool] Toggle global mapping component on/off. Only takes effect after a reset.
* `doKFReActivation`: [bool] Toggle keyframe re-activation on/off: If close to an existing keyframe, re-activate it instead of creating a new one. If false, the map will continually grow even if the camera moves in a relatively constrained area; If false, the number of keyframes will not grow arbitrarily.
* `doMapping`: [bool] Toggle entire keyframe creating / update module on/off: If false, only tracking stays active, which will prevent rapid motion or moving objects from corrupting the map.
* `useFabMap`: [bool] Use openFABMAP to find large loop-closures. Only takes effect after a reset, and requires LSD-SLAM to be compiled with FabMap.
* `allowNegativeIdepths`: [bool] Allow idepth to be (slightly) negative to avoid introducing a bias for far-away points.
* `useSubpixelStereo`: [bool] Compute subpixel-accurate stereo disparity.
* `useAffineLightningEstimation`: [bool] EXPERIMENTAL: Correct for global affine intensity changes during tracking. Might help if you have problems with auto-exposure.
* `multiThreading`: [bool] Toggle multi-threading of depth map estimation. Disable for less CPU usage, but possibly slightly less quality.
* `maxLoopClosureCandidates`: [int] Maximal number of loop-closures that are tracked initially for each new keyframe.
* `loopclosureStrictness`: [double] Threshold on reciprocal loop-closure consistency check, to be added to the map. Larger -> more (possibly wrong) loop-closures.
* `relocalizationTH`: [double] How good a relocalization-attempt has to be to be accepted. Larger -> more strict.
* `depthSmoothingFactor`: [double] How much to smooth the depth map. Larger -> less smoothing.


Useful for debug output are:

* `plotStereoImages`: [bool] Plot searched stereo lines, and color-coded stereo-results. Nice visualization of what's going on, however drastically decreases mapping speed.
* `plotTracking`: [bool] Plot final tracking residual. Nice visualization of what's going on, however drastically decreases tracking speed.
* `continuousPCOutput`: [bool] Publish current keyframe's point cloud after each update, to be seen in the viewer. Nice visualization, however bad for performance and bandwidth.







### 3.1.6 好结果的一般说明

* Use a **global shutter** camera. Using a rolling shutter will lead to inferior results.
* Use a lens with a **wide field-of-view** (we use a 130° fisheye lens).
* Use a **high framerate**, at least 30fps (depending on the movements speed of course). For our experiments, we used between 30 and 60 fps. 
* We recommend an image resolution of **640x480**, significantly higher or lower resolutions may require some hard-coded parameters to be adapted.
* LSD-SLAM is a monocular SLAM system, and as such cannot estimate the absolute scale of the map. Further it requires **sufficient camera translation**: Rotating the camera without translating it at the same time will not work. Generally sideways motion is best - depending on the field of view of your camera, forwards / backwards motion is equally good. Rotation around the optical axis does not cause any problems.
* During initialization, it is best to move the camera in a circle parallel to the image without rotating it. The scene should contain sufficient structure (intensity gradient at different depths).
* **Adjust** `minUseGrad` **and** `cameraPixelNoise` to fit the sensor-noise and intensity contrast of your camera.
* If tracking / mapping quality is poor, try decreasing the keyframe thresholds `KFUsageWeight` and `KFDistWeight` slightly to generate more keyframes.
* Note that LSD-SLAM is very much non-deterministic, i.e. results will be different each time you run it on the same dataset. This is due to parallelism, and the fact that small changes regarding when keyframes are taken will have a huge impact on everything that follows afterwards.


## 3.2 LSD-SLAM Viewer
The viewer仅用于可视化. 它也可以输出.ply格式的点云。
实时操作, 开始使用:

    rosrun lsd_slam_viewer viewer

您可以使用Rosbag记录和重新播放某些轨迹生成的输出。录制和播放使用

    rosbag record /lsd_slam/graph /lsd_slam/keyframes /lsd_slam/liveframes -o file_pc.bag
    rosbag play file_pc.bag

您不必重新启动查看器节点，它会自动重置图形。

如果只想将某个点云从.bag文件引导到查看器中，可以使用

    rosrun lsd_slam_viewer viewer file_pc.bag



### 3.2.1 可用的热键

- `r`: Reset, will clear all displayed data.

- `w`: Print the number of points / currently displayed points / keyframes / constraints to the console.

- `p`: Write currently displayed points as point cloud to file lsd_slam_viewer/pc.ply, which can be opened e.g. in meshlab. Use in combination with sparsityFactor to reduce the number of points.


### 3.2.2 参数 (Dynamic Reconfigure)

- `showKFCameras `: Toggle drawing of blue keyframe camera-frustrums. min: False, default: True, max: True
- `showKFPointclouds `: Toggle drawing of point clouds for all keyframes. min: False, default: True, max: True
- `showConstraints `: Toggle drawing of red/green pose-graph constraints. min: False, default: True, max: True
- `showCurrentCamera `: Toggle drawing of red frustrum for the current camera pose. min: False, default: True, max: True
- `showCurrentPointcloud `: Toggle drawing of the latest point cloud added to the map. min: False, default: True, max: True
- `pointTesselation `: Size of points. min: 0.0, default: 1.0, max: 5.0
- `lineTesselation `: Width of lines. min: 0.0, default: 1.0, max: 5.0
- `scaledDepthVarTH `: log10 of threshold on point's variance, in the respective keyframe's scale. min: -10.0, default: -3.0, max: 1.0
- `absDepthVarTH `: log10 of threshold on point's variance, in absolute scale. min: -10.0, default: -1.0, max: 1.0
- `minNearSupport `: Only plot points that have #minNearSupport similar neighbours (higher values remove outliers). min: 0, default: 7, max: 9
- `cutFirstNKf `: Do not display the first #cutFirstNKf keyframe's point clouds, to remove artifacts left-over from the random initialization. min: 0, default: 5, max: 100
- `sparsifyFactor `: Only plot one out of #sparsifyFactor points, selected at random. Use this to significantly speed up rendering for large maps. min: 1, default: 1, max: 100
- `sceneRadius `: Defines near- and far clipping plane. Decrease to be able to zoom in more. min: 1, default: 80, max: 200
- `saveAllVideo `: Save all rendered images... only use if you know what you are doing. min: False, default: False, max: True
- `keepInMemory `: If set to false, the point cloud is only stored in OpenGL buffers, and not kept in RAM. This greatly reduces the required RAM for large maps, however also prohibits saving / dynamically changing sparsifyFactor and variance-thresholds. min: False, default: True, max: True





# 4 数据集

为了方便起见，我们提供了许多数据集，包括视频、LSD-Slam的输出和生成的.ply格式的点云。详情请见
[http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)


# 5 许可
LSD-SLAM根据GNU通用公共许可版本3（GPLv3）获得许可，请参阅http://www.gnu.org/licenses/gpl.html。

出于商业目的，我们还提供不同许可条款下的专业版本。



# 6 故障排除 / FAQ

**How can I get the live-pointcloud in ROS to use with RVIZ?**

You cannot, at least not on-line and in real-time. The reason is the following:

In the background, LSD-SLAM continuously optimizes the pose-graph, i.e., the poses of all keyframes. Each time a keyframe's pose changes (which happens all the time, if only by a little bit), all points from this keyframe change their 3D position with it. Hence, you would have to continuously re-publish and re-compute the whole pointcloud (at 100k points per keyframe and up to 1000 keyframes for the longer sequences, that's 100 million points, i.e., ~1.6GB), which would crush real-time performance.

Instead, this is solved in LSD-SLAM by publishing keyframes and their poses separately:
- keyframeGraphMsg contains the updated pose of each keyframe, nothing else.
- keyframeMsg contains one frame with it's pose, and - if it is a keyframe - it's points in the form of a depth map.

Points are then always kept in their keyframe's coodinate system: That way, a keyframe's pose can be changed without even touching the points. In fact, in the viewer, the points in the keyframe's coodinate frame are moved to a GLBuffer immediately and never touched again - the only thing that changes is the pushed modelViewMatrix before rendering. 

Note that "pose" always refers to a Sim3 pose (7DoF, including scale) - which ROS doesn't even have a message type for.

If you need some other way in which the map is published (e.g. publish the whole pointcloud as ROS standard message as a service), the easiest is to implement your own Output3DWrapper.


**Tracking immediately diverges / I keep getting "TRACKING LOST for frame 34 (0.00% good Points, which is -nan% of available points, DIVERGED)!"**
- double-check your camera calibration.
- try more translational movement and less roational movement


