# 使用方法

このリポジトリの利用手順は以下の通りです。

## 前提条件
- NVIDIA GPU 環境（`--gpus all` オプションを使用）
- [Isaac Sim](https://developer.nvidia.com/isaac-sim) がインストール済み
- ROS 2 環境（`rviz2`, `teleop_twist_keyboard` 利用可能）
- Docker がインストール済み
- 環境設定は以下を参照: [yolo_ros GitHub リポジトリ](https://github.com/mgonzs13/yolo_ros)

## 手順

1. **Isaac Sim を起動**  
   `scene` フォルダ内の `scene.usd` を開きます。

2. **RViz 2 を起動**  
   ```bash
   rviz2
