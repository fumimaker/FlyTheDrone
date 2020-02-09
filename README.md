# FlyTheDrone
放棄されてしまったDroneKitを使ってArdupilotを制御してみるリポジトリ．

# Requirement
- Python2系
    - 動作はPython2.7.3で確認してます．
- Raspi3B
    - 違くても動くと思いますがRaspiのバージョンによってピン配列が異なる場合もある．Pythonが動けばなんでもいい．
- Pixhawk1
    - Temelが取れればなんでもいいです．

# helloDrone.py
Raspiをドローンに乗せて，オンボード制御をするための入門です．
1. RaspiのUARTポート(RXD1, TXD1)をあらかじめ設定しておきます．Raspi-configを行いGPUクロックを固定するかBluetoothを停止して割り当てる作業が必要です．  
2. Pixhawk~~のパチモン~~のTelem2とRaspiUARTピンを繋げます．TXRXはクロスするように．RaspiとTemel2の電源をつなぐとPixhawkから2Aくらい引き出そうとしてPixhawkが落ちるので注意．Raspiの電源はBECなど別途用意してください．GNDは基準を合わせるのに必要なので繋いでください．Pihawk
がTelem2じゃなくても4,5を設定すれば使えます．自分は空いてたのでそのままtelem2を使ってます．通信早いほうがいいと思ったので設定でTelem2のBaudrateを57600から115200にあげてます．920kとか動くのかな．マイコンが処理できなそう．
