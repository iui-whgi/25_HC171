sudo chmod 666 /dev/ttyACM*

lerobot-replay \
  --robot.type=bi_so101_follower \
  --robot.left_arm_port=/dev/ttyACM0 \
  --robot.right_arm_port=/dev/ttyACM1 \
  --robot.id=dual_so101 \
  --dataset.repo_id=JisooSong/sandwich_996to1000_vhlv \
  --dataset.episode=4