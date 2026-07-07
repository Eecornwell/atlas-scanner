#!/bin/bash
# Step /dev/ptp0 to system clock time repeatedly until the offset is small
# enough for phc2sys to track with frequency adjustment alone (<50ms).
# The igc PHC free-runs fast so a single set is not enough.

PHC=/dev/ptp0
PHC_CTL=/usr/sbin/phc_ctl
MAX_ITER=120
THRESHOLD_NS=50000000  # 50ms

for i in $(seq 1 $MAX_ITER); do
    $PHC_CTL $PHC set 2>/dev/null
    sleep 0.05
    PHC_T=$($PHC_CTL $PHC get 2>/dev/null | grep -oP '[\d]+\.[\d]+'  | head -1)
    SYS_T=$(date +%s.%N)
    if [ -z "$PHC_T" ]; then sleep 0.1; continue; fi
    OFFSET_NS=$(awk "BEGIN { d=($SYS_T-$PHC_T); if(d<0)d=-d; printf \"%d\", d*1e9 }")
    echo "phc_step[$i]: offset=${OFFSET_NS}ns"
    if [ "$OFFSET_NS" -lt "$THRESHOLD_NS" ] 2>/dev/null; then
        echo "PHC within threshold — handing off to phc2sys"
        exit 0
    fi
    sleep 0.05
done
echo "Warning: step loop exhausted"
exit 0
