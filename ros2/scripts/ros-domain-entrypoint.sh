#!/usr/bin/env bash
set -euo pipefail

domain="${ROS_DOMAIN_ID:-76}"
if [[ ! "$domain" =~ ^[0-9]{1,3}$ ]]; then
  printf 'ros-domain-entrypoint: ROS_DOMAIN_ID must be an integer from 1 through 232\n' >&2
  exit 2
fi
domain_number=$((10#$domain))
if ((domain_number < 1 || domain_number > 232)); then
  printf 'ros-domain-entrypoint: ROS_DOMAIN_ID=0 is disabled; select a domain from 1 through 232\n' >&2
  exit 2
fi
export ROS_DOMAIN_ID="$domain_number"

exec /ros_entrypoint.sh "$@"
