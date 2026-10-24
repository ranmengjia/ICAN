# 地面 Nav2 导航说明

`ican_inspection_navigation` 在用户版中以二进制安装包形式提供，只包含地面 Nav2 配置。

## 启动

```bash
bash scripts/run_inspection_env.sh
```

## Nav2 输入

- `/map`
- `/scan`
- `/odom`
- `map -> odom -> base_footprint -> laser_link`

## Nav2 输出

- `/cmd_vel`

麦克纳姆轮底盘支持 `linear.x`、`linear.y` 和 `angular.z`，Nav2 控制器参数已按全向底盘配置。

## 用户程序如何调用

用户程序可以调用 ROS 2 action：

```text
navigate_to_pose
nav2_msgs/action/NavigateToPose
```

最小模板 `ican_inspection_examples/minimal_user_node.py` 中的 `send_ground_nav_goal()` 展示了如何发送地面目标点。

## Livox 3D 雷达

实体两栖无人机如果使用 Livox 3D 激光雷达，地面 Nav2 可以使用 `pointcloud_to_laserscan` 将 `PointCloud2` 转成 `/scan`。需要安装：

```bash
sudo apt install ros-humble-pointcloud-to-laserscan
```
