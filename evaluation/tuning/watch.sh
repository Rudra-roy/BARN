#!/usr/bin/env bash
#
# Live view of a running BARN campaign (or a single tuning batch).
#
#     evaluation/tuning/watch.sh            # refresh every 5 s
#     evaluation/tuning/watch.sh 2          # every 2 s
#     evaluation/tuning/watch.sh --once     # one snapshot, pipe-friendly
#
# Layout: finished worlds condensed to one line each at the top, the world
# currently running shown trial by trial at the bottom. During a 10 hour, 500
# trial campaign the useful question is "how is the world running RIGHT NOW
# doing, and is the overall number holding" -- not a fixed 10-row window.
#
# Reads files only; never touches the run.

set -o pipefail
TUNING_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${TUNING_DIR}/../.." && pwd)"

ONCE=no
[ "${1:-}" = "--once" ] && { ONCE=yes; shift; }
INTERVAL="${1:-5}"

render() {
  # Prefer an active campaign; fall back to the newest tuning batch.
  local camp results kind label
  camp=$(ls -dt "${REPO_ROOT}"/results/*/public_* 2>/dev/null | head -1)
  # A campaign that has started but not yet written its first result must still
  # win over a stale tuning batch, or the monitor shows yesterday's numbers for
  # the first few minutes of a ten hour run.
  local camp_age=999999 batch_age=999999 newest_batch
  [ -n "$camp" ] && camp_age=$(( $(date +%s) - $(stat -c %Y "$camp" 2>/dev/null || echo 0) ))
  newest_batch=$(ls -t "${TUNING_DIR}"/world_*/*/raw_results.txt 2>/dev/null | head -1)
  [ -n "$newest_batch" ] && batch_age=$(( $(date +%s) - $(stat -c %Y "$newest_batch" 2>/dev/null || echo 0) ))
  if [ -n "$camp" ] && [ "$camp_age" -lt "$batch_age" ]; then
    results="${camp}/raw_results.txt"; kind=campaign; label=$(basename "$camp")
    if [ ! -s "$results" ]; then
      printf '\033[1mBARN campaign\033[0m  %s   %s\n\n' "$label" "$(date +%T)"
      printf '  started %dm ago -- world 0, trial 1 running, no result written yet\n' "$((camp_age/60))"
      return
    fi
  elif [ -n "$camp" ] && [ -s "${camp}/raw_results.txt" ]; then
    results="${camp}/raw_results.txt"; kind=campaign; label=$(basename "$camp")
  else
    results=$(ls -t "${TUNING_DIR}"/world_*/*/raw_results.txt 2>/dev/null | head -1)
    [ -z "$results" ] && { echo "  no run found yet"; return; }
    kind=batch
    label="$(basename "$(dirname "$(dirname "$results")")")/$(basename "$(dirname "$results")")"
  fi

  local started now elapsed
  started=$(stat -c %Y "$results" 2>/dev/null)
  # Campaign start is better approximated by the directory, which is created first.
  [ "$kind" = campaign ] && started=$(stat -c %Y "$camp" 2>/dev/null)
  now=$(date +%s); elapsed=$(( now - started ))

  printf '\033[1mBARN %s\033[0m  %s   %s\n\n' "$kind" "$label" "$(date +%T)"

  awk -v elapsed="$elapsed" -v kind="$kind" '
    { w=$1; ok=$2; col=$3; to=$4; at=$5; sc=$6
      n[w]++; s[w]+=sc; k[w]+=ok; c[w]+=col; t[w]+=to
      if (!(w in seen)) { seen[w]=1; order[++nw]=w }
      last=w
      # keep the current world trial-by-trial
      cur_at[w "," n[w]]=at; cur_sc[w "," n[w]]=sc
      cur_ok[w "," n[w]]=ok; cur_col[w "," n[w]]=col; cur_to[w "," n[w]]=to
      tot++; totscore+=sc; totok+=ok; totcol+=col; totto+=to
    }
    END {
      # ---- finished worlds, condensed ----
      done_n=0
      line=""
      for (i=1; i<=nw; i++) {
        w=order[i]
        if (w==last && n[w]<10) continue           # still running
        done_n++
        entry=sprintf("w%-3s %.3f %d/%d", w, s[w]/n[w], k[w], n[w])
        if (c[w]>0) entry=entry "!"
        line=line sprintf("  %-20s", entry)
        if (done_n%4==0) { print "  " line; line="" }
      }
      if (line!="") print "  " line
      if (done_n>0) print ""

      printf "  \033[1mcompleted %d worlds, %d trials\033[0m   running mean \033[1m%.4f\033[0m", done_n, tot, (tot?totscore/tot:0)
      printf "   success %d/%d", totok, tot
      if (totcol>0) printf "   \033[1;31m%d COLLISION\033[0m", totcol
      if (totto>0)  printf "   \033[33m%d timeout\033[0m", totto
      print ""

      # ---- current world, trial by trial ----
      if (n[last]<10 || done_n==0) {
        printf "\n  \033[1m>> world %s -- trial %d of 10 running\033[0m\n", last, n[last]+1
        printf "     %-6s %-9s %-9s %s\n", "trial", "AT", "score", "outcome"
        for (j=1; j<=n[last]; j++) {
          key=last "," j
          if (cur_col[key]==1)     out="\033[1;31mCOLLISION\033[0m"
          else if (cur_to[key]==1) out="\033[33mtimeout\033[0m"
          else if (cur_ok[key]==1) out="ok"
          else                     out="fail"
          printf "     %-6d %-9.1f %-9.4f %s\n", j, cur_at[key], cur_sc[key], out
        }
        if (n[last]>0) printf "     %-6s %-9s %-9.4f world so far\n", "", "", s[last]/n[last]
      }

      # ---- pace ----
      if (tot>0) {
        per=elapsed/tot
        rem=(500-tot)*per
        printf "\n  elapsed %dh%02dm   %.0f s/trial   est remaining %dh%02dm  (%d/500)\n",
               elapsed/3600, (elapsed%3600)/60, per, rem/3600, (rem%3600)/60, tot
      }
    }
  ' "$results"
}

while true; do
  [ "$ONCE" = no ] && clear
  render
  [ "$ONCE" = yes ] && break
  printf '\n  refreshing every %ss -- Ctrl-C to exit\n' "$INTERVAL"
  sleep "$INTERVAL"
done
