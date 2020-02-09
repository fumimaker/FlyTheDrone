#!usr/bin/env python
# -*- coding: utf-8 -*-
print( "dronekitスタート" )    # 開始メッセージ

# 必要なライブラリをインポート
from kbhit import *                 # kbhitを使うために必要(同じフォルダにkbhit.pyを置くこと)
from subprocess import Popen        # subprocessの中から、Popenをインポート
from signal import signal, SIGINT   # Ctrl+C(SIGINT)の送出のために必要 
from dronekit import connect        # connectを使いたいのでインポート
from dronekit import VehicleMode    # VehicleModeも使いたいのでインポート
from dronekit import LocationGlobal, LocationGlobalRelative   # ウェイポイント移動に使いたいのでインポート
import time                         # ウェイト関数time.sleepを使うために必要


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)


# kbhit()を使うための「おまじない」を最初に２つ書く
atexit.register(set_normal_term)
set_curses_term()

# dronekit SITL の起動情報
# example: 'dronekit-sitl copter --home=35.079624,136.905453,50.0,3.0 --instance 0'
sitl_frame          = 'copter'          # rover, plane, copterなどのビークルタイプ
sitl_home_latitude  = '35.894087'       # 緯度(度)  柏の葉キャンパス駅前ロータリー
sitl_home_longitude = '139.952447'      # 経度(度)
sitl_home_altitude  = '17.0'            # 高度(m)
sitl_home_direction = '0.0'             # 機首方位(度)
sitl_instance_num   = 0                 # 0〜

# コマンドライン入力したい文字列をリスト形式で作成
sitl_boot_list = ['dronekit-sitl',sitl_frame,
                  '--home=%s,%s,%s,%s' % (sitl_home_latitude,sitl_home_longitude,sitl_home_altitude,sitl_home_direction),
                  '--instance=%s'%(sitl_instance_num)]

print '# sitl command: ', sitl_boot_list        # 文字列を表示
p = Popen(sitl_boot_list)   # サブプロセスの起動
time.sleep(1)   # 起動完了のために1秒待つ

#connection_stringの生成
connection_string = 'tcp:localhost:' + str(5760 + int(sitl_instance_num) * 10 ) # インスタンスが増えるとポート番号が10増える

# フライトコントローラ(FC)へ接続
print( "FCへ接続: %s" % (connection_string) )    # 接続設定文字列を表示
vehicle = connect(connection_string, wait_ready=True)    # 接続

#Ctrl+cが押されるまでループ
try:
    while True:
        if kbhit():     # 何かキーが押されるのを待つ
            key = getch()   # 1文字取得

            # keyの中身に応じて分岐
            if key=='g':                # guided
                vehicle.mode = VehicleMode( 'GUIDED' )
            elif key=='l':              # land
                vehicle.mode = VehicleMode( 'LAND' )
            elif key=='a':              # arm
                vehicle.armed = True
            elif key=='d':              # disarm
                vehicle.armed = False
            elif key=='t':              # takeoff
                vehicle.simple_takeoff(alt=10)
            elif key=='1':              # simple_goto
                # 柏の葉キャンパス交番上空30mへ
                point = LocationGlobalRelative( 35.893246, 139.954909 , 30 )
                vehicle.simple_goto(point)
            elif key=='2':              # simple_goto
                # 三井ガーデンホテル上空50mへ
                point = LocationGlobalRelative( 35.895236, 139.952468 , 50 )
                vehicle.simple_goto(point)
            elif key=='r':              # RTL
                vehicle.mode = VehicleMode( 'RTL' )
            elif key=='f':
                arm_and_takeoff(10)
                print "wait for 10sec..."
                time.sleep(10)
                print "Landing",
                vehicle.mode = VehicleMode("LAND")
                while vehicle.armed:
                    print ".",
                    time.sleep(1)
                print ""
                print "Landed. Close connection..."
                vehicle.close()
                print "completed."


        # ここはif文と同じインデントなので，キーに関係なく1秒に1回実行される
        # 現在の状態を表示
        print("--------------------------" )
        print(" System status: %s" % vehicle.system_status.state)
        print(" Is Armable?: %s" % vehicle.is_armable)
        print(" Armed: %s" % vehicle.armed) 
        print(" Mode: %s" % vehicle.mode.name )
        print(" GlobalLocation: %s" % vehicle.location.global_frame)
        print(" Altitude: %s" % vehicle.location.global_relative_frame.alt)
        time.sleep(1)

except( KeyboardInterrupt, SystemExit):    # Ctrl+cが押されたら離脱
    print( "SIGINTを検知" )

# フライトコントローラとの接続を閉じる
vehicle.close()

 # サブプロセスにもSIGINT送信
p.send_signal(SIGINT)
p.communicate()
time.sleep(1)   # 終了完了のために1秒待つ

print("終了．")    # 終了メッセージ
