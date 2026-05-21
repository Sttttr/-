define laser ranger (
  sensor(
    range [ 0.0  10.0 ]
    fov 270
    samples 360
  )
  color "blue"
  size [ 0.05 0.05 0.1 ]
)

define my_robot position (
  size [0.5 0.5 0.25]  # 初稿中提到的机器人参数
  origin [0 0 0 0]
  gui_nose 1
  drive "diff"         # 差速模型
  laser(pose [ 0 0 0 0 ])
)

define floorplan model (
  color "gray30"
  boundary 1
  gui_nose 0
  gui_grid 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  laser_return 1
)