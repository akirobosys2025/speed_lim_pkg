# speed_lim_pkg
このリポジトリはcmd_velを監視して速度制限や緊急停止を行うためのROS2のパッケージです．

## lim_node
入力された速度指令を監視し，設定された最大速度を超えないよう制限を行います．  
また，非常停止要求があった場合は即座に速度を0にします．  

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
lim_nodeでは，速度制限の処理を値だけでなく，運転状態を３つの状態として管理しています．

| 状態 | 説明 |
|---|---|
| `NORMAL` | 速度制限が発生していない通常状態 |
| `LIMITED` | 最大速度を超えたため制限が発生している状態 |
| `EMERGENCY` | 非常停止が有効になっている状態 |

## 開発環境
- OS: Ubuntu 22.04 LTS
- 開発言語: Python 3.10

## LICENSE
- このソフトウェアパッケージは，３条項BSDライセンスの下，再頒布および使用が許可されます．
- このパッケージには[上田隆一の作成したコンテナ](https://hub.docker.com/repository/docker/ryuichiueda/ubuntu22.04-ros2)(© Ryuichi Ueda)を利用したものが含まれます．
- © 2025 Aki Moto
