#!/usr/bin/env bash
# cargo-fuzz runner (of-ipt.15).
#
# One place that knows how a target is invoked, so a local run and a CI run are
# the same command. What it encapsulates:
#
#   * the corpus layout — libFuzzer's *first* directory argument is the one it
#     writes new inputs into, and every later one is read-only. So the working
#     corpus comes first and the committed seeds come second, which is what
#     keeps `fuzz/seeds/` from filling up with machine-generated inputs.
#   * the STEP target's extra seed source: the kernel's real AP203 test files
#     are the best seeds available and far too large to duplicate under
#     fuzz/seeds/, so libFuzzer is pointed at them where they already live.
#   * the resource limits. Without `-rss_limit_mb` a single pathological input
#     can OOM the runner and report as an unrelated failure.
#
# Usage:
#   tools/fuzz/run.sh <target> [seconds]
#   tools/fuzz/run.sh step_parse 300
#
# Targets: step_parse, topology_check, nurbs_eval.
# Requires a nightly toolchain and cargo-fuzz:
#   rustup toolchain install nightly && cargo +nightly install cargo-fuzz --locked
set -euo pipefail

TARGET="${1:?usage: tools/fuzz/run.sh <target> [seconds]}"
SECONDS_TO_RUN="${2:-60}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

case "$TARGET" in
step_parse | topology_check | nurbs_eval) ;;
*)
	echo "unknown fuzz target: $TARGET" >&2
	echo "expected one of: step_parse, topology_check, nurbs_eval" >&2
	exit 2
	;;
esac

CORPUS="fuzz/corpus/$TARGET"
SEEDS="fuzz/seeds/$TARGET"
mkdir -p "$CORPUS" "fuzz/artifacts/$TARGET"

# Read-only extra corpora, most specific first.
EXTRA=("$SEEDS")
if [ "$TARGET" = "step_parse" ]; then
	# Every directory of the kernel's STEP test data that actually holds a
	# .stp, discovered rather than listed: libFuzzer does not recurse into
	# subdirectories, and the corpus gains new ones over time (of-ipt.16 added
	# occ/{blend,coincident,nurbs,periodic,tangent,thin}). Enumerating them
	# here keeps a newly added edge-case file seeded without anyone having to
	# remember this file.
	#
	# The .stp filter matters: sibling directories such as reference/ hold
	# per-file JSON oracles, and libFuzzer takes every file in a corpus
	# directory as a seed. Seeding the STEP parser with JSON is not harmful,
	# but it spends the corpus budget on inputs that are rejected in the first
	# few bytes.
	while IFS= read -r dir; do
		EXTRA+=("$dir")
	done < <(find crates/opensolid-kernel/tests/data/step -name '*.stp' \
		-exec dirname {} \; | sort -u)
fi

echo "fuzzing $TARGET for ${SECONDS_TO_RUN}s"
echo "  writable corpus: $CORPUS"
echo "  read-only seeds: ${EXTRA[*]}"

exec cargo +nightly fuzz run "$TARGET" "$CORPUS" "${EXTRA[@]}" -- \
	-max_total_time="$SECONDS_TO_RUN" \
	-rss_limit_mb=4096 \
	-timeout=25 \
	-print_final_stats=1
