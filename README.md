＜自己位置推定の基本操作＞
> ssm-coordinator
> ssm-monitor
> ./config
> sudo ssm-joystick-handler -d /dev/input/js0

> cd ./script
> sudo ./str2str_ssm_golf_reg.sh
> sudo ./gnss-f9p-handler -d /dev/ttyUSB1

> sudo ./imu-handler -d /dev/ttyACM0

> ./localizer

＜WP生成の基本操作＞
＜自己位置推定の基本操作＞を実施
> sudo ./recordOperation -d /dev/ttyUSB0
> ./logger.sh
> ./createWPfile

＜手動操縦の基本操作＞
> ssm-coordinator
> ssm-monitor
> ./config
> sudo ssm-joystick-handler -d /dev/input/js0
> sudo ./manualOperation -d /dev/USB0

＜自律走行の基本操作＞
＜自己位置推定の基本操作＞を実施
> sudo ./navigate -d /dev/ttyUSB0
> ./navi-viewer


＜コマンド群＞
config          : 各種パラメータ設定
config-viewer   : ログから設定ファイルGM_Config.cfgを作成

createWPfile    : WPファイルを作成

detectObstacle  : 障害物検知
detectObstacle-viewer: 障害物検知確認用ビューワー

gnss-f9p-handler: 2周波GNSS Z9P受信機用ハンドラー
gnss-f9p-viewer : GNSS用ビューワー

imu-handler     : RTのIMU用ハンドラー
imu-viewer      : IMU用ビューワー

manualOperation : 手動操縦。ハンドル、アクセル、レバーが操作可能。
handle-viewer   : ハンドル用ビューワー
accel-viewer    : アクセル用ビューワー
lever-viewer    : レバー用ビューワー
recordOperation : モータフリーモードでログのみSSMへ送信（CreateWP用プログラム）

localizer       : 自己位置推定
localizer-viewer: 自己位置推定結果確認用ビューワー

navigate        : 自律走行プログラム

urg-handler     : 北陽製LiDAR用ハンドラー
urg-viewer      : PCD確認用ビューワー

navi-viewer     : ナビゲーション用ビューワー
control-log2txt : 制御変数の保存
WP-viewer       : WP確認用ビューワー
vel-viewer      : 速度確認用ビューワー
