#!/bin/bash

# 定义需要运行的文件夹编号列表
# folders=("1" "2" "3" "4" "31" "33" "34" "50" "51" "52" "54"  "50" "51" "52" "54"  "50" "51" "52" "54")
folders=("50" "51" "52" "54"  "50" "51" "52" "54")

# 基础路径配置
BASE_DATASET="/home/robotlab/dataset/7floor_chair"
VOCAB="Vocabulary/ORBvoc.bin"
CONFIG="configs/7floor_chair.yaml"
MAP_OUTPUT="map/self/GroundObjects"

# 循环开始
for folder in "${folders[@]}"
do
    echo "------------------------------------------------"
    echo "正在处理序列: $folder"
    echo "------------------------------------------------"

    # 执行程序
    ./dsp_slam_rgbd \
        "$VOCAB" \
        "$CONFIG" \
        "$BASE_DATASET/$folder" \
        "$BASE_DATASET/$folder/associations.txt" \
        "$MAP_OUTPUT"

    # 可选：如果需要在每次运行之间暂停一下，或者清理缓存，可以在这里添加指令
    echo "序列 $folder 处理完成。"
done

echo "所有任务已执行完毕！"