前もって作ったETラリーの計画(全4872配置、plan_<配置>.json を plans.zip にまとめたもの)。
ロボットは --rally-strategy local(と PC から届かなかったとき)に、ここの plans.zip から計画を選ぶ
(robot_program/et_rally_plan_select.py。探す順: ET_RALLY_PLANS_DIR → ここ → ~/et_rally_plans)。

2026-11-17 版: ガレージの条件 横10cm・縦12cm(g1012)、往復をまとめる手直し(plan_tidy)入り。
抜けて戻る通過の 7.2cm(pb72、235配置の作り直し)はまだ入っていない。
作り方: wireless_device/et_rally_curve_proto/export_plans.py <計算結果のフォルダ> --out <出力フォルダ>
