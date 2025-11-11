#!/usr/bin/env bash
set -euo pipefail

BAGDIR="$1"                     # 例: ~/ros2_ws/data/bags/2025-11-03_12-00-00_expA
OUTDIR="data/converted/${1##*/}_mcap"
PRESET="${STORAGE_PRESET:-zstd_small}"
LOCK="data/converted/.convert.lock"

mkdir -p data/converted

# 二重起動防止（簡易ロック）
exec 9>"$LOCK"
flock -n 9 || { echo "[INFO] another convert running, queueing..."; flock 9; }

# 既存ならユニーク化
[ -e "$OUTDIR" ] && OUTDIR="${OUTDIR}_$(date +%Y%m%d-%H%M%S)"

TMPYAML="$(mktemp)"
cat > "$TMPYAML" <<EOF
output_bags:
  - uri: ${OUTDIR}
    storage_id: mcap
    storage_preset_profile: ${PRESET}
EOF

echo "[INFO] convert start: $BAGDIR -> $OUTDIR (preset=$PRESET)"
# 低優先度で変換
nice -n 10 ionice -c2 -n7 ros2 bag convert --input "$BAGDIR" --output-options "$TMPYAML"
rm -f "$TMPYAML"

# 簡易健全性チェック
if [ -f "${OUTDIR}/metadata.yaml" ] && ls "${OUTDIR}"/*.mcap >/dev/null 2>&1; then
  echo "[INFO] convert ok: $OUTDIR"
else
  echo "[ERROR] convert failed: $OUTDIR"
  exit 1
fi

# 成功後元データ削除
echo "[INFO] deleting source: ${BAGDIR}"
rm -rf --one-file-system -- "${BAGDIR}"

