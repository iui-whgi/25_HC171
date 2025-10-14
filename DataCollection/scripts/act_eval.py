#!/usr/bin/env python3
import torch
import time
import os
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.robots import make_robot_from_config

def main():
    # Hugging Face에서 훈련된 모델을 로드 (config 일부만 반영 + safetensors 가중치 로드)
    from huggingface_hub import hf_hub_download
    import json
    import safetensors.torch as st
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.configs.types import NormalizationMode, PolicyFeature, FeatureType

    print("Hugging Face에서 훈련된 모델을 불러오는 중...")

    # 1) 기본 ACTConfig를 명시 (입출력 특징 유지)
    config = ACTConfig()
    config.device = "cuda" if torch.cuda.is_available() else "cpu"
    config.input_features = {
        "observation.images.left": PolicyFeature(type=FeatureType.VISUAL, shape=[3, 480, 640]),
        "observation.images.top": PolicyFeature(type=FeatureType.VISUAL, shape=[3, 480, 640]),
        "observation.images.right": PolicyFeature(type=FeatureType.VISUAL, shape=[3, 480, 640]),
        "observation.images.front": PolicyFeature(type=FeatureType.VISUAL, shape=[3, 480, 640]),
        "observation.state": PolicyFeature(type=FeatureType.STATE, shape=[12])
    }
    config.output_features = {
        "action": PolicyFeature(type=FeatureType.ACTION, shape=[12])
    }
    # 정규화는 데이터셋 통계가 없으므로 비활성화
    config.normalization_mapping = {
        "VISUAL": NormalizationMode.IDENTITY,
        "STATE": NormalizationMode.IDENTITY,
        "ACTION": NormalizationMode.IDENTITY
    }

    # 2) HF의 config.json에서 아키텍처 핵심 키만 선택 반영
    local_dir = "/home/son/lerobot_0914/sandwich-checkpoint_20000"
    if os.path.isdir(local_dir):
        cfg_path = os.path.join(local_dir, "config.json")
    else:
        cfg_path = hf_hub_download("iui-whgi/sandwich-checkpoint_20000", filename="config.json")

    with open(cfg_path, "r") as f:
        hf_cfg = json.load(f)

    allowed_keys = [
        "chunk_size", "n_action_steps", "vision_backbone", "pretrained_backbone_weights",
        "replace_final_stride_with_dilation", "pre_norm", "dim_model", "n_heads",
        "dim_feedforward", "feedforward_activation", "n_encoder_layers", "n_decoder_layers",
        "use_vae", "latent_dim", "n_vae_encoder_layers", "dropout", "kl_weight",
    ]
    for k in allowed_keys:
        if k in hf_cfg:
            setattr(config, k, hf_cfg[k])

    # 3) 정책 생성 후 safetensors 가중치 로드
    policy = ACTPolicy(config, dataset_stats=None)
    if os.path.isdir(local_dir):
        model_path = os.path.join(local_dir, "model.safetensors")
    else:
        model_path = hf_hub_download("iui-whgi/sandwich-checkpoint_20000", filename="model.safetensors")
    state = st.load_file(model_path)
    result = policy.load_state_dict(state, strict=False)
    try:
        missing_cnt = len(result.missing_keys)
        unexpected_cnt = len(result.unexpected_keys)
    except Exception:
        missing_cnt, unexpected_cnt = 0, 0
    print(f"가중치 로드: missing={missing_cnt}, unexpected={unexpected_cnt}")

    policy.eval()
    if torch.cuda.is_available():
        policy = policy.to("cuda")
        print("모델이 GPU로 이동되었습니다.")
    print("✅ 훈련된 ACT 모델 로드 완료 (Hugging Face)")
    
    print("ACT 모델이 로드되었습니다!")
    print(f"디바이스: {'cuda' if torch.cuda.is_available() else 'cpu'}")
    print("모델 정보: iui-whgi/sandwich-checkpoint_20000")
    
    # 로봇 설정 (카메라 포함)
    from lerobot.robots.bi_so101_follower.config_bi_so101_follower import BiSO101FollowerConfig
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    
    robot_config = BiSO101FollowerConfig(
        id="dual_so101",
        left_arm_port="/dev/ttyACM0",
        right_arm_port="/dev/ttyACM1",
        cameras={
            # 감지된 4개 카메라를 각 뷰에 매핑
            "left": OpenCVCameraConfig(index_or_path="/dev/video8", width=640, height=480, fps=30),
            "top": OpenCVCameraConfig(index_or_path="/dev/video13", width=640, height=480, fps=30),
            "right": OpenCVCameraConfig(index_or_path="/dev/video11", width=640, height=480, fps=30),
            "front": OpenCVCameraConfig(index_or_path="/dev/video2", width=640, height=480, fps=30),
        }
    )
    robot = make_robot_from_config(robot_config)
    
    # 로봇 연결
    print("로봇 연결 중...")
    try:
        robot.connect()
        print("로봇 연결 성공!")
    except Exception as e:
        print(f"로봇 연결 실패: {e}")
        return
    
    # 제어 루프
    print("샌드위치 스태킹 시작!")
    try:
        while True:
            try:
                obs = robot.get_observation()
            except Exception as e:
                print(f"관측 실패: {e}. 우측 팔 재연결 시도...")
                try:
                    robot.right_arm.disconnect()
                except Exception:
                    pass
                try:
                    robot.right_arm.connect(calibrate=False)
                    print("우측 팔 재연결 성공")
                except Exception as e2:
                    print(f"우측 팔 재연결 실패: {e2}. 다음 루프로 계속")
                time.sleep(0.1)
                continue
            
            # 관측값을 정책이 기대하는 형식으로 변환
            policy_obs = {}
            
            # 카메라 이미지들을 텐서로 변환하고 GPU로 이동 (HWC -> CHW)
            device = "cuda" if torch.cuda.is_available() else "cpu"
            # 없는 뷰는 front 이미지로 대체
            front_img = obs["front"]
            img_left = obs.get("left", front_img)
            img_top = obs.get("top", front_img)
            img_right = obs.get("right", front_img)
            img_front = front_img

            policy_obs["observation.images.left"] = torch.tensor(img_left, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(device)
            policy_obs["observation.images.top"] = torch.tensor(img_top, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(device)
            policy_obs["observation.images.right"] = torch.tensor(img_right, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(device)
            policy_obs["observation.images.front"] = torch.tensor(img_front, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(device)
            
            # 로봇 상태를 텐서로 변환하고 GPU로 이동 (12개 관절 위치)
            robot_state = torch.tensor([
                obs["left_shoulder_pan.pos"], obs["left_shoulder_lift.pos"], obs["left_elbow_flex.pos"],
                obs["left_wrist_flex.pos"], obs["left_wrist_roll.pos"], obs["left_gripper.pos"],
                obs["right_shoulder_pan.pos"], obs["right_shoulder_lift.pos"], obs["right_elbow_flex.pos"],
                obs["right_wrist_flex.pos"], obs["right_wrist_roll.pos"], obs["right_gripper.pos"]
            ], dtype=torch.float32).unsqueeze(0).to(device)
            policy_obs["observation.state"] = robot_state
            
            with torch.no_grad():
                action = policy.select_action(policy_obs)
            # 액션 통계 출력 (디버깅)
            try:
                action_cpu = action.detach().float().cpu().abs()
                a_mean = float(action_cpu.mean().item())
                a_max = float(action_cpu.max().item())
                print(f"action| mean={a_mean:.4f} max={a_max:.4f}")
            except Exception:
                pass
            
            # 액션을 로봇 형식으로 변환 (현재 위치 기준으로 작은 움직임)
            robot_action = {}
            
            # 현재 관절 위치를 기준으로 움직임 적용 (정규화 단위: 바디[-100,100], 그리퍼[0,100])
            body_gain = 50.0
            body_clip = 20.0
            gripper_gain = 50.0
            gripper_clip = 20.0
            for i, joint_name in enumerate([
                "left_shoulder_pan.pos", "left_shoulder_lift.pos", "left_elbow_flex.pos",
                "left_wrist_flex.pos", "left_wrist_roll.pos", "left_gripper.pos",
                "right_shoulder_pan.pos", "right_shoulder_lift.pos", "right_elbow_flex.pos",
                "right_wrist_flex.pos", "right_wrist_roll.pos", "right_gripper.pos"
            ]):
                # 액션 텐서 차원 유연 처리
                if hasattr(action, "ndim") and action.ndim == 2:
                    model_val = action[0, i]
                elif hasattr(action, "ndim") and action.ndim == 1:
                    model_val = action[i]
                else:
                    model_val = action[i] if isinstance(action, (list, tuple)) else 0.0
                try:
                    model_action = float(model_val.item())
                except Exception:
                    model_action = float(model_val)
                current_position = obs[joint_name]
                if joint_name.endswith("gripper.pos"):
                    delta = max(-gripper_clip, min(gripper_clip, model_action * gripper_gain))
                    target = current_position + delta
                    target = max(0.0, min(100.0, target))
                else:
                    delta = max(-body_clip, min(body_clip, model_action * body_gain))
                    target = current_position + delta
                    target = max(-100.0, min(100.0, target))
                robot_action[joint_name] = target
            
            print(f"현재 위치 기준: 작은 움직임만 실행")
            robot.send_action(robot_action)
            
            # 10Hz 제어 (0.1초 간격)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("제어 종료")
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()