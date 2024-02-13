# OmniD435_ROS

## 環境構築

ワークスペースのsrcディレクトリ内にクローンしてください。  
例:
```bash
cd ~/catkin_ws/src
git clone git@github.com:suzakutakumi/OmniD435_ROS.git
```

## ビルド

```bash
cd ~/catkin_ws
catkin_build
```

## 実行

事前準備
```bash
cd ~/catkin_ws
source devel/setup.bash
```

publisherとsubscriberの両方を起動
```bash
roslaunch omni-d435 omni-d435.launch
```

publisherのみ起動
```bash
roslaunch omni-d435 omni-d435.launch pub:=true
```

subscriberのみ起動
```bash
roslaunch omni-d435 omni-d435.launch sub:=true
```
