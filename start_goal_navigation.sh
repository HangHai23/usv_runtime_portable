#!/usr/bin/env bash
set -Eeo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$PROJECT_DIR/tools/start_standalone.sh" navigation "${1:-right}"
