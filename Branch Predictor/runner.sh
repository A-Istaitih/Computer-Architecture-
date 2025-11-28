#!/bin/bash

# Paths
TEST_DIR="tests"
ACTUAL_DIR="actual_outputs"
EXEC="./bp_main"

# Ensure output directory exists
mkdir -p "$ACTUAL_DIR"
testNumber=0
counter=0
# Run and compare all tests
for trc_file in "$TEST_DIR"/*.trc; do
    base=$(basename "$trc_file" .trc)
    out_file="$TEST_DIR/$base.out"
    actual_out="$ACTUAL_DIR/$base.out"
    ((testNumber ++))

    echo "== Running $base =="

    if [[ ! -f "$out_file" ]]; then
        echo "Missing expected output for $base"
        continue
    fi

    # Run the predictor
    $EXEC "$trc_file" > "$actual_out"
    if [[ $? -ne 0 ]]; then
        echo "❌ Error running $base"
        continue
    fi

    # Compare
    if diff -u "$out_file" "$actual_out" > /dev/null; then
        echo "✅ PASS"
        ((counter ++))
    else
        echo "❌ FAIL (diff below)"
        #diff -u "$out_file" "$actual_out"
        #exit 1
    fi
done
echo "testNumber = $testNumber"
echo "number of pass    = $counter"
