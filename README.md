# speed_lim_pkg
このリポジトリはcmd_velを監視して速度制限や緊急停止を行うためのROS2のパッケージです．

## 導入
次の手順でリポジトリをダウンロードできます．  

'''  
# ワークスペースを作成し，srcへ移動してリポジトリをダウンロードしてください．
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone git@github.com:akirobosys2025/speed_lim_pkg.git
'''  

## lim_node
入力された速度指令を監視し，設定された最大速度を超えないよう制限を行う．  
また，非常停止要求があった場合は即座に速度を0にする．  

### Subscribe
| トピック名 | 型 | 説明 |
|----------|----|----|
| `/cmd_vel` | geometry_msgs/Twist | 元の速度指令 |
| `/emergency_stop` | std_msgs/Bool | 非常停止フラグ |

### Publish
| トピック名 | 型 | 説明 |
|----------|----|----|
| `/cmd_vel_safe` | geometry_msgs/Twist | 制限後の速度指令 |
| `/speed_limit_reason` | std_msgs/String | 速度制限の理由 |
| `/speed_limit_status` | std_msgs/String | 現在の状態 |

### 状態
lim_nodeでは，速度制限の処理を値だけでなく，運転状態を３つの状態として管理している．

| 状態 | 説明 |
|---|---|
| `NORMAL` | 速度制限が発生していない通常状態 |
| `LIMITED` | 最大速度を超えたため制限が発生している状態 |
| `EMERGENCY` | 非常停止が有効になっている状態 |

## LICENSE
- このソフトウェアパッケージは，３条項BSDライセンスの下，再頒布および使用が許可されます．
- © 2025 Aki Moto
