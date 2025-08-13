# 使用方法

このリポジトリの利用手順は以下の通りです。

## 前提条件
- NVIDIA GPU 環境（`--gpus all` オプションを使用）
- [Isaac Sim](https://developer.nvidia.com/isaac-sim) がインストール済み
- ROS 2 環境（`rviz2`, `teleop_twist_keyboard` を使用）
- Docker がインストール済み
- 環境設定は以下を参照: [yolo_ros](https://github.com/mgonzs13/yolo_ros)

## 手順

1. **Isaac Sim を起動**  
   `scene` フォルダ内の `scene.usd` を開きます。

2. **RViz 2 を起動**  
   ```bash
   rviz2
   
3. **YOLO ROS コンテナを起動** 
   ```bash
   sudo docker run -it --rm --gpus all yolo_ros

4. **キーボードでロボットを操作** 
   別のターミナルで以下を実行し、Isaac Sim 内のロボットをキーボードで制御します。
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard

## その他

本プロジェクトでは、本来GPTを介して自然言語によるIsaac Sim上のロボット制御や画像認識を試みましたが、現時点ではまだ実現できていません。現在開発途中ですが、プログラムは deprecated/intelligent_move に記載しています。
