# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

超音波センサ(HC-SR04 ×4)で人を検知して追従する移動台車(2-5班)のファームウェア。マイコンは
STM32F446RET6 (NUCLEO-F446RE)、STM32CubeIDE (CubeMX生成コード + HAL) ベース。

- 制作物の目的・制約・要件は `docs/requirements_by_part.md` にまとまっている。
- ゴール: 対象物との距離を保って追従し、近接/見失いで停止、液晶で状態表示、ブザーで報知する。

## ⚠️ ドキュメントの優先順位(重要)

`docs/` には4つの文書があるが、**鮮度が違う**。矛盾したら新しい方(v0.1)を正とすること。

1. **`docs/requirements_by_part.md`** (v0.1, 2026-11-02) — **最新・正**。部品別要件・決定事項(D-01〜D-14)・
   TODOリスト。実装前に必ずここを読む。
2. `docs/software_requirements.md` (v0.5, 2026-09-29) — 旧版。R43(LED板で3状態表示)など
   `requirements_by_part.md` で上書きされた記述が残っている(例: 状態は「追従中/停止中」の2つのみで
   「異常」状態は追加しない、表示器はLED板ではなく液晶ILI9341)。
3. `docs/pin_assignment.md` (v0.2, 2026-11-02) — ピン配置は実質最新(Core/Inc/main.h の `*_Pin` 定義と
   一致している)が、文書自体は `requirements_by_part.md` から「参照しない」扱いにされている。ピン配置の
   一次ソースは Notion「マイコンピン対応表」。
4. `docs/distance_process.md` (2026-11-13) — 距離データ処理(`Distance_Process`)の**実装**メモ。
   Notion「超音波計測 機能モジュール表」と食い違う箇所は、こちら(実装)を正とする。

`requirements_by_part.md` 内の `D-xx`(決定事項)と `SOFT-xx`/`US-xx`/`BZ-xx` などの部品別ID、および
末尾の TODO セクションが、実装すべきことと既知の未決事項(保留中の設計判断)の一覧になっている。新しい
制御ロジックを書く前に、関連するIDの行を確認すること。

## リポジトリ構成

ファームウェアは2つの同名フォルダに分かれて存在するが、**実体があるのはリポジトリ直下のみ**:

- `Core/`, `Drivers/`, `*.ioc`, `.cproject`, `.project`, `*.ld` — **本体**。STM32CubeIDE プロジェクト
  (CubeMX生成コード + HAL)。`Core/Src/main.c` がエントリポイント、`Core/Src/ultrasonic.c` /
  `Core/Inc/ultrasonic.h` が実装済みの超音波計測ドライバ、`Core/Src/distance_process.c` /
  `Core/Inc/distance_process.h` が距離データ処理(位置推定)。
- `Ultrasonic-Tracking-Vehicle/` — **空のCMakeスキャフォールド**(未追跡/未コミット)。`Src/main.c` は
  25行のテンプレートのままで、`ultrasonic.c` も `Core/`/`Drivers/` も存在しない。開発対象ではない。
  紛らわしいので触らないこと。
- `matlab/` — 台車の速度/位置制御のゲイン設計用 Simulink モデル(`cart_cascade_PID.slx` など)と、
  モータ諸元を導出する `cart_plant_params.m`。ファームウェアとは独立した制御設計の作業場。
- `Claude outputs/`, `feedback_block_diagrams.pptx` — 設計検討やレポートの成果物。コードではない。

## ビルド

CubeMXで `.ioc` を編集したときは、CubeIDE で "Generate Code" するか、CubeMX CLI で再生成してから
`Core/Src/*.c` の `USER CODE` ブロック内のみを編集する(それ以外はCubeMXが上書きする)。

- STM32CubeIDE: プロジェクトを開いて通常どおりビルド/デバッグ(st-link経由)。
- `.github/workflows/ci.yml` は `platformio.ini` を前提に `pio test -e native` と
  `pio run -e nucleo_f446re` を実行するが、**リポジトリに `platformio.ini` が存在しない**(PlatformIO環境は
  まだ移行/整備されていない)。CI は現状このままでは動かない可能性が高い。PlatformIO化を前提にした変更を
  する場合は、まず `platformio.ini` と `test/` の追加が必要。

テストフレームワークやネイティブ単体テストは未整備(`test/` ディレクトリなし)。

## アーキテクチャ

`docs/requirements_by_part.md` の SOFT-01 により、ソフトウェアは4層構成を志向している
(表示・操作 → 状態・判定 → 制御 → ドライバ、依存は上から下の一方向)。**現時点で実装済みなのは
ドライバ層の超音波計測と、制御層の距離データ処理(位置推定)**。状態管理・追従制御(位置制御/向き制御/
速度制御)・ブザー・本番のモータ制御は未実装。`lcd.c`(液晶の簡易ドライバ)と `motor_encoder_test.c`
(起動時に 't' で入るテストメニュー)は動作確認用のコード。`main()` は接続確認後、距離を周期的に取得し、
`Distance_Process` の結果(det / X / Y)とともに USART2(115200bps)に printf するだけのデモループ。

### 超音波計測 (`Core/Src/ultrasonic.c`)

- HC-SR04 ×4、センサ番号 0:外側左 1:内側左 2:内側右 3:外側右。隣接センサの相互干渉を避けるため
  2個ずつの組(組0=センサ0・2、組1=センサ1・3)で**交互に発火**(60msごと、各センサは120ms更新)。
- Trig はGPIO出力でソフトウェアが10µsパルスを生成、Echo はEXTI両エッジ割り込み + TIM5(1µsフリーラン
  タイマ、Prescaler=83 @84MHz)のタイムスタンプ差でパルス幅→距離(mm = pulse_us × 0.343 / 2)を算出。
  `HAL_GPIO_EXTI_Callback` → `Ultrasonic_EXTI_Callback` に配線されている。
- 公開APIは `Ultrasonic_Init` / `Ultrasonic_Trigger(pair_id)` / `Ultrasonic_OnEcho` /
  `Ultrasonic_GetResult` / `Ultrasonic_EXTI_Callback` / `Ultrasonic_CheckConnection`。
  距離・応答なしフラグは `volatile` 配列に保持し、読み出し/組の切り替え時のみ `__disable_irq`/
  `__enable_irq` で保護する(ロックフリーに近い設計、ミューテックス等は使わない)。
- `s_pin[]`(センサ番号→Trig/Echoピン)と `s_pair_mask[]`(組→センサ)がハードウェア結線の唯一の定義
  箇所。配線変更時はここだけ直す。

### 距離データ処理 (`Core/Src/distance_process.c`)

- 制御層。`Ultrasonic_GetResult` で得た4個の距離を `Distance_Process(dist, &UsResult)` に渡す。
  20〜4000mm を有効とし、300mm 以内で `detect=1`、有効2個以上でガウス・ニュートン法により対象の位置
  (X, Y)を推定する(`estimated`、返り値 0/-1/-2)。センサ位置 `DP_SENSOR_X_MM`/`DP_SENSOR_Y_MM` は
  図面から読んだ仮の値、初回の追従距離 `DP_INIT_FOLLOW_MM`(500mm)も仮の値。
- **Notion の仕様と食い違う点・既知の制限(残差検査なし、有効1個では推定しない等)は
  `docs/distance_process.md` が正**(実装を優先して書いてある)。実機での X・Y の確認は未。

### ピン配置の要点(`Core/Inc/main.h` の `*_Pin` 定義が正)

モータ1(PA6/PA7)はTIM3との競合を避けるためTIM13/TIM14へ逃がしている、エンコーダ1(PC8/PC9)は
ハードウェアエンコーダモード不可のためEXTIソフト4逓倍が必要、など複数のピン競合の回避策が
`docs/pin_assignment.md` の「4. 設計上の問題点と対処」に説明されている。新しいペリフェラルを
.iocに追加する前にこのセクションを読むこと。

## コミット規約

コミットメッセージは日本語の Conventional Commits 形式 (`feat:`, `docs:`, `refactor:` など) ＋
関連Issue番号を末尾に `(#N)` で付与する(例: `refactor: fire の Trig 操作を set_trig と
HAL_GPIO_WritePin にまとめる (#15)`)。
