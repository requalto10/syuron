#!/bin/bash
cnf=$2
num_trial=$3
exec_file=$1

for ((i=0; i<$num_trial; i++)); do
(
    ./$exec_file $cnf $i

    # プログラム正常終了の確認
    if [ "$?" -ne 0 ]; then
        echo "Execution failed."
        exit 1
    fi
) >> tmp &
done

wait

sat_count=$(awk '{count += gsub(/SAT/, "", $2)} END {print count}' tmp)
unsat_count=$(awk '{count += gsub(/UNSAT/, "", $2)} END {print count}' tmp)
avg_iteration=$(awk '{sum += $3} END {print sum/NR}' tmp)
avg_duration=$(awk '{sum += $4} END {print sum/NR}' tmp)

cat tmp | sort -n

echo ""
echo "summary:"
echo "SAT: $sat_count"
echo "UNSAT: $unsat_count"
echo "AVG_ITERATION: $avg_iteration"
echo "AVG_DURATION: $avg_duration"

rm tmp
