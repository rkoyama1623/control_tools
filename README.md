# 環境変数の設定
`export ROBOT=HRP2JSKNT`

# 起動方法
```
roseus "(jsk)" "(rbrain)"
load "play-hrpsysfile.l" ;;this repository file
```  


# ログファイルの保存からローカルへのコピーまで　
```
send *ri* :start-log
save-log :fname "test"
```
save-log関数でログファイルがscpされて，ローカルの
~/Copy/Documents/log/<ROBOT_NAME>/test_yyyy_MM-dd_hh-mm_ss
に保存される．

# 使い方がわからなくなったら
```
load "play-hrpsysfile.l" ;;this repository file
usage
```

# 結果の描画
```
plot-rs :mode :ee-pos :axis 2 :robot hrp2jsknt-robot
```
