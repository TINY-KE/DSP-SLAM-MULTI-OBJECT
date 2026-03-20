###
 # @Author: GetOverMassif 164567487@qq.com
 # @Date: 2022-10-11 13:50:58
 # @LastEditors: GetOverMassif 164567487@qq.com
 # @LastEditTime: 2022-10-13 19:14:25
 # @FilePath: /DSP-SLAM/scripts/run_redwood.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 


# ./dsp_slam_rgbd  \
# Vocabulary/ORBvoc.bin  \
# configs/ruihai.yaml  \
# /home/robotlab/dataset/Ruihan/circle_desk \
# /home/robotlab/dataset/Ruihan/circle_desk/associations.txt \
# map/self/GroundObjects

# ./dsp_slam_rgbd  \
# Vocabulary/ORBvoc.bin  \
# configs/ruihai.yaml  \
# /home/robotlab/dataset/Ruihan/desk-moniter \
# /home/robotlab/dataset/Ruihan/desk-moniter/associations.txt \
# map/self/GroundObjects

# ./dsp_slam_rgbd  \
# Vocabulary/ORBvoc.bin  \
# configs/ruihai.yaml  \
# /home/robotlab/dataset/Ruihan/mylivingroom \
# /home/robotlab/dataset/Ruihan/mylivingroom/associations.txt \
# map/self/GroundObjects

# ./dsp_slam_rgbd  \
# Vocabulary/ORBvoc.bin  \
# configs/ruihai.yaml  \
# /home/robotlab/dataset/Ruihan/mybedroom \
# /home/robotlab/dataset/Ruihan/mybedroom/associations.txt \
# map/self/GroundObjects


# ./dsp_slam_rgbd  \
# Vocabulary/ORBvoc.bin  \
# configs/7floor_chair.yaml  \
# /home/robotlab/dataset/7floor_chair/1 \
# /home/robotlab/dataset/7floor_chair/1/associations.txt \
# map/self/GroundObjects



# 配置参数
VOC="Vocabulary/ORBvoc.bin"
CONFIG="configs/ruihai_charpt5.yaml"
DATA="/home/robotlab/dataset/ruihai_charpt5/6_2"
ASSOC="/home/robotlab/dataset/ruihai_charpt5/6_2/associations.txt"
SAVE="map/self/GroundObjects"

for i in {1..5}
do
    echo "========================================"
    echo "开始第 $i 次实验运行..."
    
    # 执行程序
    ./dsp_slam_rgbd $VOC $CONFIG $DATA $ASSOC $SAVE
    
    echo "第 $i 次实验已完成。"
    sleep 2  # 每次运行间隙停顿2秒，防止系统资源未释放
done