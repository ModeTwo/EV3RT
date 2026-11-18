本番2回目(攻める走り)用の計画。全4872配置、plan_<配置>.json を plans.zip にまとめたもの。
使い方(ロボットで、2026-Alpha で): ET_RALLY_PLANS_DIR=et_rally_plans_m20 python3 alpha.py <left|right> --mission full --lap-bottle-aim --rally-strategy local
ログの「precomputed et_rally_plans_m20/plans.zip:plan_...」で、こちらが使われたと分かる。

2026-11-19 版: すき間2cm(HR_MARGIN_LIST=2.0,1.5)で全配置を計算し直し、持ち時間70秒で周回が1つ増える1161配置だけを
2cm 版に差し替えた。残り3711配置は et_rally_plans/plans.zip(3cm)と同じ。
差し替えた配置の点検(接触・禁止エリア・ゲートの順・抜けて戻る7.2cm・ガレージ・切り上げ・配置の鍵): NG 0。
抜けて戻る通過が7.2cm に足りない2配置(r0414-b4243-y0313 7.18、r3343-b4142-y2131 7.14)は差し替えていない。
