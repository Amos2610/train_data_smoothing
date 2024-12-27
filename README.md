# train_data_smoothing
学習させた関節角度のデータをstompにPublishするパッケージ

### インストール
ワークスペースのsrc以下で
```
git clone https://github.com/Amos2610/train_data_smoothing.git
```
ビルドをする
```
catkin build
source <your_ws>/devel/setup.bash
```

### 実行環境
+ Ubuntu20.04LTS
+ ROS Noetic
+ Python3.8.10

※ ROS Noeticのインストールは[こちら](https://qiita.com/vivy/items/7f8d932350175e8c0858)から

※ xArmのパッケージインストール&セットアップは[こちら](https://github.com/hiroarkMani/xArm_manipulation)から

### パッケージの中身
```r
    .
    ├── csv_files ・・・関節データを含むcsvファイル
    │   └── after_train.csv
    ├── launch
    │   └── task.launch ・・・ xArm6の起動を担う（各パラメータ，プランニングアルゴリズムの設定等できるようになっている）
    ├── msg
    │   └── PathSeed.msg ・・・パスシードのトピック通信に必要なmsgファイル
    └── scripts
        ├── tools ・・・使えそうなプログラムを置いておきました
        │   ├── ex_and_con_cartesian_all.py ・・・C空間 から T空間への変換
        │   ├── random_sg.py ・・・ランダムなスタートとゴール姿勢を発生させるプログラム
        │   └── visualization.py ・・・T空間のeefの軌道の可視化
    >>> └── train_path_pub.py ・・・ 学習させた関節角度のデータをstompにPublishするファイル
    ├── CMakeLists.txt 
    ├── package.xml
    └── README.md
```
### 実行方法
1. ターミナルで以下のコマンドでGazeboとMoveIt!を立ち上げる

    ```
    roslaunch train_data_smoothing task.launch
    ```
2. 関節角度のデータをstompへPublishする
   
    ```bash
    rosrun train_data_smoothing train_path_pub.py
    ```
3. 手動もしくはxArm6を動かすようなscriptでxArm6を動かす
   stompになっている，学習させたパスの通りになっていることを確認
