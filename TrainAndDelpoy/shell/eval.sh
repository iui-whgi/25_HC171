sudo chmod 666 /dev/ttyACM*

python examples/SO-100/eval_lerobot.py \
    --robot.type=bi_so101_follower \
    --robot.left_arm_port=/dev/ttyACM0 \
    --robot.right_arm_port=/dev/ttyACM1 \
    --robot.id=dual_so101 \
    --robot.cameras="{ 
        left: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30}, 
        right: {type: opencv, index_or_path: 6, width: 640, height: 480, fps: 30}, 
        top: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}, 
        front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}
    }" \
    --policy_host=localhost \
    --policy_port=5555 \
    --lang_instruction="pick the bread, place it on dish. pick the ham, place it on bread. pick the bread, place it on vegetable."
