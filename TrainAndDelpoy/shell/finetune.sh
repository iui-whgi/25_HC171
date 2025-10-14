export HF_PATH=../../.cache/huggingface/hub/datasets--JisooSong--sandwich_

nohup python scripts/gr00t_finetune.py \
    --dataset-path ${HF_PATH}<dataset_name>/snapshots/* \
    --output_dir ./output/full_sandwich_dataset \
    --num-gpus 2 \
    --data_config so101_4cam \
    --dataloader_num_workers 12 \
    --video_backend torchvision_av \
    --max-steps 10000 \
    --save-steps 1000
    > nohup.out
