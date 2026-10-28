#!/usr/bin/env bash
set -euo pipefail

REPO_URL=https://github.com/stonet-research/cheops25-IO-characterization-of-LLM-model-kv-cache-offloading-nvme
COMMIT=489095d069e7d6eac2e7154d99412edc584c1492
TRACE_PATH=results/figure5-6-kv-offloading-flexgen/flexgen-kv-offload-opt-6.7b-bs-64-ext4-trace
TRACE=opt-6.7b-kv-offload-bs-64-ext4-bpftrace-block.txt
TRACE_SHA256=ea318b9b238a44f61433f4c9f4d62e4eb0f0960cb4628a560474dfb2903448b8
TRACE_SIZE=1069223555
SMALL_FILES=(
	LICENSE
	bpftrace-scripts/bio-bite-size.bt
	"$TRACE_PATH/opt-6.7b-kv-offload-bs-64-ext4.txt"
	"$TRACE_PATH/flexgen-opt-6.7b-kv-trace-ext4-bs-64-log.txt"
)

OUT=${1:-"$(cd "$(dirname "$0")" && pwd)/cheops25"}
REPO_DIR="$OUT/repo"
mkdir -p "$OUT"

if [ ! -d "$REPO_DIR/.git" ]; then
	GIT_LFS_SKIP_SMUDGE=1 git clone --quiet --no-checkout "$REPO_URL" "$REPO_DIR"
fi
GIT_LFS_SKIP_SMUDGE=1 git -C "$REPO_DIR" -c advice.detachedHead=false checkout --quiet "$COMMIT"

POINTER=$(git -C "$REPO_DIR" cat-file -p "$COMMIT:$TRACE_PATH/$TRACE")
grep -qx "oid sha256:$TRACE_SHA256" <<<"$POINTER" || { echo "LFS pointer oid differs from $TRACE_SHA256" >&2; exit 1; }
grep -qx "size $TRACE_SIZE" <<<"$POINTER" || { echo "LFS pointer size differs from $TRACE_SIZE" >&2; exit 1; }

for f in "${SMALL_FILES[@]}"; do
	cp "$REPO_DIR/$f" "$OUT/$(basename "$f")"
done

if [ -f "$OUT/$TRACE" ] && [ "$(sha256sum "$OUT/$TRACE" | cut -d' ' -f1)" = "$TRACE_SHA256" ]; then
	echo "$TRACE already present"
else
	if git lfs version >/dev/null 2>&1; then
		git -C "$REPO_DIR" lfs pull --include="$TRACE_PATH/$TRACE"
		cp "$REPO_DIR/$TRACE_PATH/$TRACE" "$OUT/$TRACE.part"
	else
		curl -fL --retry 3 -o "$OUT/$TRACE.part" \
			"https://media.githubusercontent.com/media/stonet-research/cheops25-IO-characterization-of-LLM-model-kv-cache-offloading-nvme/$COMMIT/$TRACE_PATH/$TRACE"
	fi
	[ "$(stat -c %s "$OUT/$TRACE.part")" = "$TRACE_SIZE" ] || { echo "size mismatch" >&2; exit 1; }
	[ "$(sha256sum "$OUT/$TRACE.part" | cut -d' ' -f1)" = "$TRACE_SHA256" ] || { echo "sha256 mismatch" >&2; exit 1; }
	mv "$OUT/$TRACE.part" "$OUT/$TRACE"
fi

echo "CHEOPS'25 trace ready in $OUT"
ls -l "$OUT"
