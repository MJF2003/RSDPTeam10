# RSDPTeam10
AERO62520 Robotic Systems Design Project Team 10

Test using Realsense:
(in venv)

cd /home/team10/yolov5

QT_QPA_PLATFORM=xcb python infer_live_realsense_final.py \
  --weights train/blocks_bins_platform4/weights/best.pt \
  --block_clf_dir /home/team10/yolov5/bins_blocks_dataset/block_attrs/clf_out \
  --bin_clf_dir   /home/team10/yolov5/bins_blocks_dataset/bin_color/clf_out \
  --conf 0.25 \
  --show_depth
