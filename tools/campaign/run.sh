#!/usr/bin/env bash
# Campaign entry point: single-instance lock around campaign.py so a cron
# firing while a long run is still going becomes a no-op instead of a pile-up.
set -euo pipefail
cd "$(dirname "$0")/../.."
exec flock -n "${TMPDIR:-/tmp}/opensolid-campaign.lock" \
    python3 tools/campaign/campaign.py "$@"
