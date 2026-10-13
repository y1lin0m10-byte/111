# Third-Party Notices

The following ROS-Industrial packages are vendored under `src/` to provide the
official KUKA KR 6 R900-2 geometry and meshes:

```text
kuka_resources
kuka_kr6_support
kuka_kr10_support
```

Source:

```text
https://github.com/ros-industrial/kuka_experimental
branch: melodic-devel
```

The KR 6 R900-2 URDF macro uses the KR 10 R900-2 mesh set because
ROS-Industrial documents that those robot variants have identical appearance.
The package metadata has been converted from catkin to ROS 2 `ament_cmake`.

License information is retained in each vendored package's `package.xml`.

