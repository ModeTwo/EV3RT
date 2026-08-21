import sys  # sys モジュールを読み込む
import argparse  # argparse モジュールを読み込む
import time  # time モジュールを読み込む
import threading  # threading モジュールを読み込む
import signal  # signal モジュールを読み込む
import math  # math モジュールを読み込む
from enum import IntEnum, Enum, auto  # enum から必要なクラス・関数を読み込む
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor  # etrobo_python から必要なクラス・関数を読み込む
from simple_pid import PID  # simple_pid から必要なクラス・関数を読み込む
from py_trees.trees import BehaviourTree  # py_trees.trees から必要なクラス・関数を読み込む
from py_trees.behaviour import Behaviour  # py_trees.behaviour から必要なクラス・関数を読み込む
from py_trees.common import Status  # py_trees.common から必要なクラス・関数を読み込む
from py_trees.composites import Sequence  # py_trees.composites から必要なクラス・関数を読み込む
from py_trees.composites import Selector  # py_trees.composites から必要なクラス・関数を読み込む
from py_trees.composites import Parallel  # py_trees.composites から必要なクラス・関数を読み込む
from py_trees.common import ParallelPolicy  # py_trees.common から必要なクラス・関数を読み込む
from py_trees import (  # py_trees から必要なクラス・関数を読み込む
    display as display_tree,  # 直前の定義・関数呼び出しに渡す値を指定する
    logging as log_tree  # この処理を実行する
)  # 直前から続く定義・引数・リストを閉じる
from py_etrobo_util import Video, TraceSide, TargetInterested, Plotter, SymmetricClamper, Color, ColorClassifier, LowPassFilter, BottleColor, Hint, HintType  # py_etrobo_util から必要なクラス・関数を読み込む


class ReadKey(Behaviour):  # ReadKey クラスを定義する
    def __init__(self, name: str):
        super(ReadKey, self).__init__(name)  # 親クラスBehaviourの初期化処理を呼び出す
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))  # ReadKeyが初期化されたことをデバッグログに出力
        self.running = False  # まだ入力処理を開始していないことを表す
        print("init finish")

        global g_key  # この関数内で使用するグローバル変数を宣言する
        # 初回呼び出し時は処理開始状態にする
        if not self.running:
            self.running = True
            # ユーザーからの入力を受け取る
            g_key = input("Enter the given key for decryption: ") 
            # check the length of the key, it should be 4 characters long
            if len(g_key) != 4:  # 入力されたキーの長さが4文字のとき
                self.logger.warning("%+06d %s.invalid key length: %d. key should be 4 characters long."
                                    % (g_plotter.get_distance(), 
                                       self.__class__.__name__,
                                        len(g_key))
                                    )  # 警告ログを出力する
                return Status.RUNNING  # 再びキー入力を行う
            # 入力したキーを画面に表示
            print("Entered password:", g_key)
            # 入力した内容が正しか確認
            confirmation = input("Is this key correct? (y/n):")
            if confirmation.lower() == 'y': # 正しい場合
                # self.logger.info("%+06d %s.key confirmed" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                return Status.SUCCESS # 処理終了
            else:  # 正しくない場合
                # self.logger.info("%+06d %s.key rejected, please enter again" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                return Status.RUNNING # 再びキー入力を行う

    def update(self) -> Status:
        print("updatestart")
        global g_key  # この関数内で使用するグローバル変数を宣言する
        # 初回呼び出し時は処理開始状態にする
        if not self.running:
            self.running = True
            # ユーザーからの入力を受け取る
            g_key = input("Enter the given key for decryption: ") 
            # check the length of the key, it should be 4 characters long
            if len(g_key) != 4:  # 入力されたキーの長さが4文字のとき
                self.logger.warning("%+06d %s.invalid key length: %d. key should be 4 characters long."
                                    % (g_plotter.get_distance(), 
                                       self.__class__.__name__,
                                        len(g_key))
                                    )  # 警告ログを出力する
                return Status.RUNNING  # 再びキー入力を行う
            # 入力したキーを画面に表示
            print("Entered password:", g_key)
            # 入力した内容が正しか確認
            confirmation = input("Is this key correct? (y/n):")
            if confirmation.lower() == 'y': # 正しい場合
                self.logger.info("%+06d %s.key confirmed" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                return Status.SUCCESS # 処理終了
            else:  # 正しくない場合
                self.logger.info("%+06d %s.key rejected, please enter again" % (g_plotter.get_distance(), self.__class__.__name__))  # 動作状況を情報ログとして出力する
                return Status.RUNNING # 再びキー入力を行う


if __name__ == '__main__':  # このファイルが直接実行された場合のメイン処理を開始する
    print("ここまで実行")
    ReadKey(name="readkey")